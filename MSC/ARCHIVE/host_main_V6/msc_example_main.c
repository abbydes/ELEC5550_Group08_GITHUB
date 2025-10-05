// boardB_bridge.c
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
#define UART_BAUD_RATE     8000000
#define UART_TX_PIN        17
#define UART_RX_PIN        18
#define UART_BUF_SIZE      2048

//frame sizes
#define RX_FRAME_SIZE (3 + 4 + 512 + 1)
static uint8_t rx_frame[3 + 4 + 512 + 1];
static uint8_t tx_frame[3 + 4 + 512 + 1];
static uint8_t sector[512];
static QueueHandle_t uart_queue;

// App queue and message types for MSC events
typedef enum { APP_DEVICE_CONNECTED, APP_DEVICE_DISCONNECTED } app_msg_id_t;
typedef struct {
    app_msg_id_t id;
    union {
        uint8_t new_dev_address;             // on connect
        msc_host_device_handle_t device;     // on disconnect
    } data;
} app_message_t;
static QueueHandle_t app_queue;

// Single-device handle
static msc_host_device_handle_t msc_dev = NULL;

// UART helpers
static void print_rx_frame(const char *tag, const uint8_t *buf, int len) {
    if (len <= 0) { ESP_LOGI(tag, "No packets received from Board A"); return; }
    char line[512]; char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) p += sprintf(p, "%02X ", buf[i]);
    *p = '\0'; ESP_LOGI(tag, "UART RX (%d bytes): %s", len, line);
}
static void print_tx_frame(const char *tag, const uint8_t *buf, int len) {
    char line[512]; char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) p += sprintf(p, "%02X ", buf[i]);
    *p = '\0'; ESP_LOGI(tag, "UART TX (%d bytes): %s", len, line);
}
static bool validate_checksum(const uint8_t *frame, int len) {
    if (len < 4) return false;
    uint8_t type = frame[1];
    uint8_t payload_len = frame[2];
    if (len != payload_len + 4) return false;
    uint32_t sum = type + payload_len;
    for (int i = 0; i < payload_len; i++) sum += frame[3 + i];
    return frame[len - 1] == (uint8_t)(0x100 - (sum & 0xFF));
}

// MSC callback: posts events to app queue
static void msc_event_cb(const msc_host_event_t *event, void *arg) {
    app_message_t msg;
    if (!app_queue) return;
    if (event->event == MSC_DEVICE_CONNECTED) {
        msg.id = APP_DEVICE_CONNECTED;
        msg.data.new_dev_address = event->device.address;
        xQueueSendFromISR(app_queue, &msg, NULL);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        msg.id = APP_DEVICE_DISCONNECTED;
        msg.data.device = event->device.handle;
        xQueueSendFromISR(app_queue, &msg, NULL);
    }
}

// USB host task
static void usb_task(void *arg) {
    ESP_LOGI(TAG, "usb_task started");

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .skip_phy_setup = false,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,   // <-- fixed typo
        .task_priority = 5,
        .stack_size = 8192,               // increased stack
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_cfg));
    ESP_LOGI(TAG, "USB Host and MSC driver installed");

    while (1) {
        esp_err_t err = usb_host_lib_handle_events(50, NULL);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
            ESP_LOGE(TAG, "usb_host_lib_handle_events() err=%s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// UART RX task
static void uart_rx_task(void *arg) {
    while (!msc_dev) {
        ESP_LOGI(TAG, "Waiting for MSC device...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "MSC device ready, starting UART processing");

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, rx_frame, sizeof(rx_frame), pdMS_TO_TICKS(100));
        if (len <= 0) continue;
        print_rx_frame(TAG, rx_frame, len);

        if (rx_frame[0] != 0xA5 || !validate_checksum(rx_frame, len)) {
            ESP_LOGW(TAG, "Invalid frame or checksum mismatch");
            continue;
        }

        uint8_t type = rx_frame[1];
        uint8_t payload_len = rx_frame[2];
        uint32_t lba = (rx_frame[3]<<24)|(rx_frame[4]<<16)|(rx_frame[5]<<8)|rx_frame[6];

        //READ10 frame...
        if (type == 'R') {
            esp_err_t ret = scsi_cmd_read10(msc_dev, sector, lba, 1, sizeof(sector));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SCSI READ10 failed");
                continue;
            }

            int payload_total = 4 + sizeof(sector);
            int total_len = 4 + payload_total;
            tx_frame[0] = 0xA5; tx_frame[1] = 'D'; tx_frame[2] = payload_total;
            tx_frame[3] = (lba>>24)&0xFF; tx_frame[4] = (lba>>16)&0xFF;
            tx_frame[5] = (lba>>8)&0xFF; tx_frame[6] = (lba>>0)&0xFF;
            memcpy(&tx_frame[7], sector, sizeof(sector));

            uint32_t sum = tx_frame[1] + tx_frame[2];
            for(int i=0; i<payload_total; i++) sum += tx_frame[3+i];
            tx_frame[total_len-1]=(uint8_t)(0x100-(sum&0xFF));

            uart_write_bytes(UART_PORT_NUM,(const char*)tx_frame,total_len);
            print_tx_frame(TAG, tx_frame, total_len);

        //WRITE10 frame...
        } else if(type == 'W') {
            memcpy(sector, &rx_frame[7], sizeof(sector));
            esp_err_t ret = scsi_cmd_write10(msc_dev, sector, lba, 1, sizeof(sector));
            if( ret != ESP_OK) {
                ESP_LOGE(TAG, "SCSI WRITE10 failed");
                continue;
            }

            uint8_t ack = 0xAA;
            uart_write_bytes(UART_PORT_NUM,&ack,1);
            print_tx_frame(TAG,&ack,1);
        }
    }
}

// Handshake
static void handshake_protocol(void) {
    const char *handshake = "Board B is ready";
    const char *expected = "Board A is ready";
    uint8_t buf[32];

    while(1) {
        uart_write_bytes(UART_PORT_NUM, handshake, strlen(handshake));
        ESP_LOGI(TAG, "Sent handshake '%s' to Board A", handshake);

        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf)-1, pdMS_TO_TICKS(1000));
        if(len>0) {
            buf[len]='\0';
            ESP_LOGI(TAG, "Received: '%s'", buf);
            if(strstr((char*)buf, expected)) break;
        }
        ESP_LOGW(TAG,"Handshake failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// App main
void app_main(void) {
    ESP_LOGI(TAG,"Board B USB MSC Host (bridge) starting...");

    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM,&uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    handshake_protocol();

    app_queue = xQueueCreate(5,sizeof(app_message_t));
    if(!app_queue) return;

    xTaskCreate(usb_task,"usb_task",8192,NULL,5,NULL);
    xTaskCreate(uart_rx_task,"uart_rx_task",8192,NULL,5,NULL);

    while(1){
        app_message_t msg;
        if(xQueueReceive(app_queue,&msg,portMAX_DELAY)==pdTRUE){
            if(msg.id==APP_DEVICE_CONNECTED){
                ESP_LOGI(TAG,"APP: Device connected -> installing (addr=%d)", msg.data.new_dev_address);
                if(msc_host_install_device(msg.data.new_dev_address,&msc_dev)!=ESP_OK){
                    msc_dev=NULL;
                }
            } else if(msg.id==APP_DEVICE_DISCONNECTED){
                if(msc_dev){
                    msc_host_uninstall_device(msc_dev);
                    msc_dev=NULL;
                }
            }
        }
    }
}
