#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/msc/msc_device.h"

static const char *TAG = "boardA_msc_bridge";

// ---------------- UART CONFIG ----------------
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     18
#define UART_BAUD_RATE  8000000
#define UART_BUF_SIZE   2048

// ---------------- TinyUSB MSC CALLBACKS ----------------
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) { return true; }
bool tud_msc_test_unit_ready_cb(uint8_t lun) { return true; }
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    *block_count = 1024;
    *block_size = 512;
}

//serial monitor functions
static void print_tx_frame(const char *tag, const uint8_t *buf, int len) {
    char line[512];
    char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) {
        p += sprintf(p, "%02X ", buf[i]);
    }
    *p = '\0';
    ESP_LOGI(tag, "UART TX (%d bytes): %s", len, line);
}

static void print_rx_frame(const char *tag, const uint8_t *buf, int len) {
    if (len <= 0) { ESP_LOGI(tag, "No data received"); return; }
    char line[512];
    char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) {
        p += sprintf(p, "%02X ", buf[i]);
    }
    *p = '\0';
    ESP_LOGI(tag, "UART RX (%d bytes): %s", len, line);
}

// Helper: build and send frame with checksum
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
    uint8_t frame[3 + payload_len + 1];
    frame[0] = 0xA5;
    frame[1] = type;
    frame[2] = payload_len;
    memcpy(&frame[3], payload, payload_len);

    // checksum = 0x100 - (sum(type + payload_len + payload) & 0xFF)
    uint32_t sum = frame[1] + frame[2];
    for (int i = 0; i < payload_len; i++) sum += payload[i];
    frame[3 + payload_len] = (uint8_t)(0x100 - (sum & 0xFF));

    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    print_tx_frame(TAG, frame, sizeof(frame));
}

int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void* buffer, uint16_t bufsize) {
    // Forward command to Board B
    //uint8_t header[3] = {0xA5, 'C', 16};
    send_frame('C', scsi_cmd, 16);
    ESP_LOGI(TAG, "Forwarded SCSI command to Board B");
    return -1;
}
// Simplified READ10 callback (waits for data back from Board B)
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    // Payload = 4 bytes LBA
    uint8_t lba_bytes[4] = {
        (uint8_t)(lba >> 24),
        (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),
        (uint8_t)(lba)
    };
    send_frame('R', lba_bytes, 4);

    ESP_LOGI(TAG, "Forwarded READ10 to Board B (LBA=%lu, len=%u)", lba, bufsize);

    // Wait for Board B to respond with 'D' frame containing sector data
    uint8_t rx_frame[3 + 4 + bufsize + 1];  // header + lba + sector + checksum
    int len = uart_read_bytes(UART_PORT_NUM, rx_frame, sizeof(rx_frame), pdMS_TO_TICKS(500));
    print_rx_frame(TAG, rx_frame, len);

    if (len >= (int)sizeof(rx_frame) && rx_frame[0]==0xA5 && rx_frame[1]=='D') {
        memcpy(buffer, &rx_frame[7], bufsize);
        return bufsize;
    }
    ESP_LOGW(TAG, "READ10 timeout or invalid response");
    return -1;
}


// Similar for WRITE10: send data + wait for ACK
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
        // Payload = 4 bytes LBA + sector data
    uint8_t payload[4 + bufsize];
    payload[0] = (uint8_t)(lba >> 24);
    payload[1] = (uint8_t)(lba >> 16);
    payload[2] = (uint8_t)(lba >> 8);
    payload[3] = (uint8_t)(lba);
    memcpy(&payload[4], buffer, bufsize);

    send_frame('W', payload, 4 + bufsize);

    ESP_LOGI(TAG, "Forwarded WRITE10 to Board B (LBA=%lu, len=%u)", lba, bufsize);

    // Wait for ACK (0xAA)
    uint8_t ack;
    int len = uart_read_bytes(UART_PORT_NUM, &ack, 1, pdMS_TO_TICKS(500));
    if (len == 1 && ack == 0xAA) return bufsize;

    ESP_LOGW(TAG, "WRITE10 failed: no ACK");
    return -1;
}

// Mandatory INQUIRY callback to fix the linker error
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    const char vid[] = "BoardA  ";
    const char pid[] = "MSC Bridge     ";
    const char rev[] = "1.0 ";
    memcpy(vendor_id, vid, sizeof(vid)-1);
    memcpy(product_id, pid, sizeof(pid)-1);
    memcpy(product_rev, rev, sizeof(rev)-1);
}

// ---------------- HANDSHAKE ----------------
static void handshake_rx_task(void *arg) {
    uint8_t buf[32];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            buf[len] = '\0';
            if (strstr((char*)buf, "Board B is ready")) {
                const char *reply = "Board A is ready";
                uart_write_bytes(UART_PORT_NUM, reply, strlen(reply));
                ESP_LOGI(TAG, "Replied to Board B handshake: '%s'", reply);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------- MAIN ----------------
void app_main(void) {
    ESP_LOGI(TAG, "Board A MSC Transparent Bridge starting...");

    // UART init
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Run handshake first
    xTaskCreate(handshake_rx_task, "   handshake_rx_task", 2048, NULL, 5, NULL);

    // Init TinyUSB
    tinyusb_config_t tusb_cfg = {0};
    //tusb_cfg.device_descriptor = NULL; // use your descriptors here
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB initialized");

    while (1) {
        tud_task(); // handle USB events
    }
}
