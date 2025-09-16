#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/msc/msc_device.h"
#include <stdint.h>
#include <string.h>

#define TAG "msc_device"

// Storage geometry
#define SECTOR_SIZE    512
#define SECTOR_COUNT   32768   // 16 MB for prototype, adjust dynamically later

// UART link to Board 1
#define UART_PORT UART_NUM_1
#define UART_TX   18   // ESP32-S3 pin
#define UART_RX   17

// Optional: remove if not used
// static SemaphoreHandle_t io_done;
// static uint8_t block_buf[SECTOR_SIZE];

// ---------------- TinyUSB MSC callbacks ------------------

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba,
                          uint32_t offset, void* buffer, uint32_t bufsize)
{
    if (offset != 0) return 0;

    uint8_t cmd[9];
    cmd[0] = 0xA1;  // custom READ opcode
    memcpy(&cmd[1], &lba, 4);
    memcpy(&cmd[5], &bufsize, 4);
    uart_write_bytes(UART_PORT, (const char*)cmd, sizeof(cmd));

    // Wait + receive data
    uart_read_bytes(UART_PORT, buffer, bufsize, pdMS_TO_TICKS(1000));
    return bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba,
                           uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
    if (offset != 0) return 0;

    uint8_t cmd[9];
    cmd[0] = 0xA2;  // custom WRITE opcode
    memcpy(&cmd[1], &lba, 4);
    memcpy(&cmd[5], &bufsize, 4);
    uart_write_bytes(UART_PORT, (const char*)cmd, sizeof(cmd));

    uart_write_bytes(UART_PORT, (const char*)buffer, bufsize);

    // Optional: wait for ack
    uint8_t ack;
    uart_read_bytes(UART_PORT, &ack, 1, pdMS_TO_TICKS(100));
    return bufsize;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4])
{
    memcpy(vendor_id,  "ESP32S3 ", 8);
    memcpy(product_id, "Bridge Disk     ", 16);
    memcpy(product_rev,"1.0", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
    *block_count = SECTOR_COUNT;
    *block_size  = SECTOR_SIZE;
}

// Correct signature for TinyUSB 5.x
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void* buffer, uint16_t bufsize)
{
    return -1; // default: unhandled
}

// ---------------- App entry ------------------

void app_main(void)
{
    // Init UART link to Board 1
    uart_config_t cfg = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Init TinyUSB
    const tinyusb_config_t tusb_cfg = {
        .external_phy = false
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB MSC bridge device started");
}
