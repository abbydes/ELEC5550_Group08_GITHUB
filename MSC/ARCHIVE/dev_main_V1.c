#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tusb.h"

static const char *TAG = "DEV";

#define UART_PORT UART_NUM_1
#define UART_TX   47
#define UART_RX   48
#define UART_BAUD 2000000
#define SECTOR_SIZE 512
static uint8_t sector_buf[SECTOR_SIZE];

void tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                       void* buffer, uint32_t bufsize) {
    uart_write_bytes(UART_PORT, (const char*)&lba, sizeof(lba));
    uart_read_bytes(UART_PORT, sector_buf, SECTOR_SIZE, portMAX_DELAY);
    memcpy(buffer, sector_buf + offset, bufsize);
}

void tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                        uint8_t* buffer, uint32_t bufsize) {
    memcpy(sector_buf + offset, buffer, bufsize);
    uart_write_bytes(UART_PORT, (const char*)&lba, sizeof(lba));
    uart_write_bytes(UART_PORT, (const char*)sector_buf, SECTOR_SIZE);
}

void app_main(void) {
    // Init UART
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT, 4096, 4096, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, -1, -1);

    ESP_LOGI(TAG, "TinyUSB MSC init...");
    tinyusb_config_t tusb_cfg = {};
    tusb_cfg.device_descriptor = NULL; // use default
    tusb_cfg.string_descriptor = NULL;
    tusb_cfg.configuration_descriptor = NULL;
    tusb_cfg.external_phy = false;
    tinyusb_driver_install(&tusb_cfg);

    while (1) {
        tud_task(); // tinyusb background
        vTaskDelay(1);
    }
}
