#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "driver/uart.h"

static const char *TAG = "HOST";

#define UART_PORT UART_NUM_1
#define UART_TX   17
#define UART_RX   18
#define UART_BAUD 2000000   // 2 Mbps link to Board 2

msc_host_device_handle_t msc_dev;

void app_main(void) {
    ESP_LOGI(TAG, "Init USB Host...");
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    // MSC Host
    msc_host_driver_config_t msc_cfg = {
        .host_driver_config = host_cfg,
        .callback = NULL,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_cfg));

    // Init UART to Board 2
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

    ESP_LOGI(TAG, "Waiting for MSC device...");
    // Simplified: wait for device
    while (msc_host_install_device(0, &msc_dev) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "MSC device mounted!");

    // Example: read first block (512 bytes)
    uint8_t buf[512];
    ESP_ERROR_CHECK(msc_host_read_sector(msc_dev, 0, buf, 1));
    ESP_LOGI(TAG, "Read sector 0, sending to Board 2...");
    uart_write_bytes(UART_PORT, (const char*)buf, 512);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
