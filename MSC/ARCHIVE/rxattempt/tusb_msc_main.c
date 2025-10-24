/*
  Rx: Transparent-ish MSC (virtual small disk) bridged to Tx over laser UART.

  - Exposes a small virtual disk (VDISK_SIZE_MB) to the PC.
  - For LDAs inside that virtual disk, READ10/WRITE10 are forwarded across UART to Tx.
  - Default UART baud = 2_000_000 (2 Mbps). Change LASER_BAUD if needed.
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb.h"

static const char *TAG = "rx_bridge";

/* ---------------------- Virtual disk config ---------------------- */
/* Expose a small virtual disk to the PC so Windows won't timeout/scan a huge volume */
#define VDISK_SIZE_MB      2                     // change to 1/4/8 as you like (keep small)
#define VDISK_BLOCK_SIZE   512
#define VDISK_BLOCK_COUNT  ((VDISK_SIZE_MB * 1024 * 1024) / VDISK_BLOCK_SIZE) // number of sectors

/* ---------------------- Laser UART config ------------------------ */
#define LASER_UART_NUM     UART_NUM_1
#define LASER_BAUD         8000000               // default 2 Mbps (change to 1000000 if needed)
#define LASER_TX_PIN       17
#define LASER_RX_PIN       18
#define LASER_BUF_SIZE     2048
#define LASER_READ_TIMEOUT_MS   1500
#define LASER_WRITE_TIMEOUT_MS  1500
#define LASER_MAX_RETRIES       3

/* ---------------------- Frame format ---------------------------- */
/*
 Frame format (Tx <-> Rx control frames)
 [0] = 0xA5
 [1] = 'R' (read) or 'W' (write)
 [2] = payload_len (N) == number of payload bytes
 [3..(3+N-1)] = payload
 [last] = checksum = (uint8_t)(0x100 - ((sum of type + payload_len + payload bytes) & 0xFF))
 For read: payload_len = 4, payload = LBA (big-endian 4 bytes)
 For write: payload_len = 4 + 512, payload = LBA + 512-byte sector
 Responses:
  - READ: Tx sends exactly 512 bytes sector (no header)
  - WRITE: Tx replies with single byte 0xAA on success
*/

/* ---------------------- Helpers --------------------------------- */
static uint8_t compute_checksum(const uint8_t *frame, int payload_len)
{
    uint32_t sum = frame[1] + frame[2];
    for (int i = 0; i < payload_len; ++i) sum += frame[3 + i];
    return (uint8_t)(0x100 - (sum & 0xFF));
}

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = LASER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(LASER_UART_NUM, LASER_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LASER_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LASER_UART_NUM, LASER_TX_PIN, LASER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Laser UART init @ %d baud (TX=%d RX=%d)", LASER_BAUD, LASER_TX_PIN, LASER_RX_PIN);
}

/* send read request (with retries) and wait for 512B sector */
static int send_read_request_and_get_sector_retry(uint32_t lba, void *buffer, uint32_t bufsize)
{
    if (bufsize != VDISK_BLOCK_SIZE) return -1;
    if (lba >= VDISK_BLOCK_COUNT) {
        // out of virtual disk range: return zeroed sector
        memset(buffer, 0, bufsize);
        return (int)bufsize;
    }

    for (int attempt = 0; attempt < LASER_MAX_RETRIES; ++attempt) {
        uint8_t frame[8];
        frame[0] = 0xA5;
        frame[1] = 'R';
        frame[2] = 4;
        frame[3] = (lba >> 24) & 0xFF;
        frame[4] = (lba >> 16) & 0xFF;
        frame[5] = (lba >> 8) & 0xFF;
        frame[6] = (lba) & 0xFF;
        frame[7] = compute_checksum(frame, 4);

        // clear old bytes, then send request
        uart_flush_input(LASER_UART_NUM);
        int wrote = uart_write_bytes(LASER_UART_NUM, (const char*)frame, sizeof(frame));
        if (wrote != sizeof(frame)) {
            ESP_LOGE(TAG, "UART write fail (read req)");
            continue;
        }

        // receive exactly bufsize bytes
        int total = 0;
        uint8_t *p = (uint8_t*)buffer;
        TickType_t start = xTaskGetTickCount();
        while (total < (int)bufsize) {
            int r = uart_read_bytes(LASER_UART_NUM, p + total, bufsize - total, pdMS_TO_TICKS(100));
            if (r > 0) total += r;
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(LASER_READ_TIMEOUT_MS)) break;
        }
        if (total == (int)bufsize) {
            return (int)bufsize;
        }
        ESP_LOGW(TAG, "Read LBA=%"PRIu32" attempt %d failed (got %d bytes)", lba, attempt+1, total);
    }
    ESP_LOGE(TAG, "Failed to read LBA=%"PRIu32, lba);
    return -1;
}

/* send write request (with retries) and wait for ACK (0xAA) */
static int send_write_request_and_wait_ack_retry(uint32_t lba, const void *buffer, uint32_t bufsize)
{
    if (bufsize != VDISK_BLOCK_SIZE) return -1;
    if (lba >= VDISK_BLOCK_COUNT) {
        // out-of-range writes: accept but ignore (pretend success)
        return (int)bufsize;
    }

    for (int attempt = 0; attempt < LASER_MAX_RETRIES; ++attempt) {
        uint8_t header[7];
        header[0] = 0xA5;
        header[1] = 'W';
        header[2] = (uint8_t)(4 + bufsize);
        header[3] = (lba >> 24) & 0xFF;
        header[4] = (lba >> 16) & 0xFF;
        header[5] = (lba >> 8) & 0xFF;
        header[6] = (lba) & 0xFF;

        // compute checksum incrementally
        uint32_t sum = header[1] + header[2];
        for (int i = 0; i < 4; ++i) sum += header[3 + i];
        const uint8_t *pbuf = (const uint8_t*)buffer;
        for (int i = 0; i < (int)bufsize; ++i) sum += pbuf[i];
        uint8_t checksum = (uint8_t)(0x100 - (sum & 0xFF));

        uart_flush_input(LASER_UART_NUM);
        if (uart_write_bytes(LASER_UART_NUM, (const char*)header, sizeof(header)) != (int)sizeof(header)) {
            ESP_LOGE(TAG, "UART write header fail");
            continue;
        }
        if (uart_write_bytes(LASER_UART_NUM, (const char*)buffer, bufsize) != (int)bufsize) {
            ESP_LOGE(TAG, "UART write payload fail");
            continue;
        }
        if (uart_write_bytes(LASER_UART_NUM, (const char*)&checksum, 1) != 1) {
            ESP_LOGE(TAG, "UART write checksum fail");
            continue;
        }

        uint8_t ack = 0;
        int r = uart_read_bytes(LASER_UART_NUM, &ack, 1, pdMS_TO_TICKS(LASER_WRITE_TIMEOUT_MS));
        if (r == 1 && ack == 0xAA) {
            return (int)bufsize;
        }
        ESP_LOGW(TAG, "Write LBA=%"PRIu32" attempt %d no ACK (r=%d, val=0x%02x)", lba, attempt+1, r, ack);
    }
    ESP_LOGE(TAG, "Failed to write LBA=%"PRIu32, lba);
    return -1;
}

/* --------------------- TinyUSB MSC callbacks --------------------- */

/* Test unit ready: always ready */
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void) lun;
    return true;
}

/* Report capacity: advertise small virtual disk to the PC */
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
    (void) lun;
    *block_count = (uint32_t)VDISK_BLOCK_COUNT;
    *block_size  = (uint16_t)VDISK_BLOCK_SIZE;
}

/* INQUIRY: advertise vendor/product/rev (local values are OK) */
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;
    // fill with padded ASCII
    memset(vendor_id, ' ', 8);
    memcpy(vendor_id, "ESP32S3", strlen("ESP32S3"));
    memset(product_id, ' ', 16);
    memcpy(product_id, "LaserBridgeVDK", strlen("LaserBridgeVDK"));
    memset(product_rev, ' ', 4);
    memcpy(product_rev, "1.0", 3);
}

/* READ10 callback -> forward via UART */
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void* buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;
    if (bufsize != VDISK_BLOCK_SIZE) {
        ESP_LOGW(TAG, "Unexpected read bufsize %u", (unsigned)bufsize);
        // attempt to handle smaller read by zeroing or returning error
        if (bufsize < VDISK_BLOCK_SIZE) memset(buffer, 0, bufsize);
        return (int32_t)bufsize;
    }

    int ret = send_read_request_and_get_sector_retry(lba, buffer, bufsize);
    if (ret < 0) {
        // On failure, return a zeroed sector so host doesn't choke (Windows may still notice)
        memset(buffer, 0, bufsize);
        return (int32_t)bufsize;
    }
    return (int32_t)ret;
}

/* WRITE10 callback -> forward via UART */
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t* buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;
    if (bufsize != VDISK_BLOCK_SIZE) {
        ESP_LOGW(TAG, "Unexpected write bufsize %u", (unsigned)bufsize);
        return -1;
    }

    int ret = send_write_request_and_wait_ack_retry(lba, buffer, bufsize);
    if (ret < 0) {
        // If write failed, return error so host retries
        return -1;
    }
    return (int32_t)ret;
}

/* SCSI fallback callback: not used for our basic bridge, return -1 (unsupported) */
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
    (void) lun;
    (void) buffer;
    (void) bufsize;
    ESP_LOGW(TAG, "Unhandled SCSI opcode 0x%02x", scsi_cmd[0]);
    return -1;
}

/* -------------------- TinyUSB descriptors ----------------------- */
/* minimal descriptors; fine for exposing the MSC interface */
static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static uint8_t const msc_fs_configuration_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(0, 0, 0x01, 0x81, 64),
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // English
    "ESP32-S3",                    // 1: Manufacturer
    "Laser MSC VDisk",             // 2: Product
    "LB123456",                    // 3: Serial
    "MSC"                          // 4: MSC
};

/* ----------------------- app_main ------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Rx bridge (virtual %d MB disk, %u sectors)",
             VDISK_SIZE_MB, (unsigned)VDISK_BLOCK_COUNT);

    uart_init();

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = msc_fs_configuration_desc,
#if (CFG_TUD_ENDOINT0_SIZE && CFG_TUD_ENDOINT0_SIZE >= 64)
        // nothing else
#endif
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB installed. The device should appear as removable disk (virtual %d MB).", VDISK_SIZE_MB);
}
