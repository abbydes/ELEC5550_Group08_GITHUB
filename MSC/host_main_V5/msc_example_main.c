// boardB_bridge.c  -- patch to use app_queue pattern and safe install
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "esp_private/msc_scsi_bot.h"

static const char *TAG = "msc_host_boardB";

// UART config
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        17
#define UART_RX_PIN        18
#define UART_BUF_SIZE      1024

static QueueHandle_t uart_queue;

// App queue for MSC events (mimics example)
typedef enum {
    APP_DEVICE_CONNECTED,
    APP_DEVICE_DISCONNECTED,
} app_msg_id_t;

typedef struct {
    app_msg_id_t id;
    union {
        uint8_t new_dev_address;             // on connect
        msc_host_device_handle_t device;     // on disconnect
    } data;
} app_message_t;

static QueueHandle_t app_queue;

// single-device handle (for your current single-device use)
static msc_host_device_handle_t msc_dev = NULL;

// checksum helper (unchanged)
static bool validate_checksum(uint8_t *frame, int len) {
    if (len < 4) return false;
    uint8_t type = frame[1];
    uint8_t payload_len = frame[2];
    if (len != payload_len + 4) return false;
    uint32_t sum = type + payload_len;
    for (int i = 0; i < payload_len; i++) sum += frame[3 + i];
    uint8_t expected = (uint8_t)(0x100 - (sum & 0xFF));
    return frame[len - 1] == expected;
}

// --- MSC callback: only send messages to the application queue (non-blocking, no install calls)
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    app_message_t msg;
    if (!app_queue) {
        ESP_LOGW(TAG, "app_queue not ready in callback");
        return;
    }

    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "msc_event_cb: MSC_DEVICE_CONNECTED (addr=%d)", event->device.address);
        msg.id = APP_DEVICE_CONNECTED;
        msg.data.new_dev_address = event->device.address;
        BaseType_t ok = xQueueSendFromISR(app_queue, &msg, NULL);
        if (ok != pdTRUE) {
            ESP_LOGW(TAG, "Failed to post DEVICE_CONNECTED to app_queue");
        }
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "msc_event_cb: MSC_DEVICE_DISCONNECTED");
        msg.id = APP_DEVICE_DISCONNECTED;
        /* note: event->device.handle may not be useful in some versions, but include it anyway */
        msg.data.device = event->device.handle;
        BaseType_t ok = xQueueSendFromISR(app_queue, &msg, NULL);
        if (ok != pdTRUE) {
            ESP_LOGW(TAG, "Failed to post DEVICE_DISCONNECTED to app_queue");
        }
    } else {
        ESP_LOGW(TAG, "msc_event_cb: unknown event: %d", event->event);
    }
}

// USB host task: installs host + MSC driver and pumps host events (don't abort on timeout)
static void usb_task(void *arg)
{
    ESP_LOGI(TAG, "usb_task started");
    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .skip_phy_setup = false,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // create background MSC task (example uses this)
    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_cfg));
    ESP_LOGI(TAG, "USB Host and MSC driver installed");

    // pump USB host lib events continuously. Ignore ESP_ERR_TIMEOUT (normal).
    while (1) {
        esp_err_t err = usb_host_lib_handle_events(50, NULL);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "usb_host_lib_handle_events() err=%s", esp_err_to_name(err));
        }
        // no msc_host_handle_events() here because create_backround_task=true spawns MSC task
        // keep loop tight so events are handled promptly
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// UART RX task: waits for msc_dev being assigned by app loop, then does raw SCSI read/write
static void uart_rx_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    uint8_t sector[512];

    while (!msc_dev) {
        ESP_LOGI(TAG, "Waiting for MSC device...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "MSC device ready, starting UART processing");

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0 && data[0] == 0xA5 && validate_checksum(data, len)) {
            if (data[1] == 'R') {
                uint32_t lba = (data[3]<<24) | (data[4]<<16) | (data[5]<<8) | data[6];
                esp_err_t ret = scsi_cmd_read10(msc_dev, sector, lba, 1, sizeof(sector));
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_PORT_NUM, (const char *)sector, sizeof(sector));
                } else {
                    ESP_LOGE(TAG, "scsi read10 failed LBA=%" PRIu32 " err=%s", lba, esp_err_to_name(ret));
                }
            } else if (data[1] == 'W') {
                uint32_t lba = (data[3]<<24) | (data[4]<<16) | (data[5]<<8) | data[6];
                memcpy(sector, &data[7], sizeof(sector));
                esp_err_t ret = scsi_cmd_write10(msc_dev, sector, lba, 1, sizeof(sector));
                if (ret == ESP_OK) {
                    uint8_t ack = 0xAA;
                    uart_write_bytes(UART_PORT_NUM, &ack, 1);
                } else {
                    ESP_LOGE(TAG, "scsi write10 failed LBA=%" PRIu32 " err=%s", lba, esp_err_to_name(ret));
                }
            }
        }
    }
}

// App main: creates queues, starts usb_task, then handles app_queue messages (install/uninstall devices)
void app_main(void)
{
    ESP_LOGI(TAG, "Board B USB MSC Host (bridge) starting...");

    // UART init
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // create the application queue for msc events
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create app_queue");
        return;
    }

    // start USB and UART tasks
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    // Main app loop: process MSC connect/disconnect messages and do msc_host_install_device() in app context
    while (1) {
        app_message_t msg;
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.id == APP_DEVICE_CONNECTED) {
                ESP_LOGI(TAG, "APP: Device connected -> installing (addr=%d)", msg.data.new_dev_address);
                esp_err_t rc = msc_host_install_device(msg.data.new_dev_address, &msc_dev);
                if (rc != ESP_OK) {
                    ESP_LOGE(TAG, "msc_host_install_device failed: %s", esp_err_to_name(rc));
                    msc_dev = NULL;
                } else {
                    ESP_LOGI(TAG, "msc_host_install_device succeeded, msc_dev=%p", (void*)msc_dev);
                }
            } else if (msg.id == APP_DEVICE_DISCONNECTED) {
                ESP_LOGI(TAG, "APP: Device disconnected");
                if (msc_dev) {
                    msc_host_uninstall_device(msc_dev);
                    msc_dev = NULL;
                }
            }
        }
    }
}

