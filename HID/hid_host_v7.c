// Transmitter: ESP32-S3 as USB HID Host → forwards HID reports over UART
// Based on ESP-IDF examples, simplified & hardened for your pipeline.

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "driver/uart.h"

#define APP_QUIT_PIN         GPIO_NUM_0

// -------- UART to receiver ----------
#define SERIAL_UART_NUM   UART_NUM_1
#define SERIAL_TX_GPIO    17
#define SERIAL_BAUD       115200
#define SERIAL_MAGIC      0xA5
// ------------------------------------

static const char *TAG = "TX";

static inline uint8_t checksum8_frame(const uint8_t *frame, size_t len_without_magic) {
    // Our frame layout: [MAGIC, TYPE, LEN, PAYLOAD..., CS]
    // CS chosen so sum(TYPE + LEN + PAYLOAD + CS) % 256 == 0
    uint32_t s = 0;
    for (size_t i = 0; i < len_without_magic; i++) s += frame[1 + i];
    return (uint8_t)((0x100 - (s & 0xFF)) & 0xFF);
}

static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(SERIAL_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SERIAL_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SERIAL_UART_NUM, SERIAL_TX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static void uart_send_framed(uint8_t type, const uint8_t *payload, size_t len) {
    if (!payload || len > 250) return;
    uint8_t hdr[3] = { SERIAL_MAGIC, type, (uint8_t)len };
    uint8_t cs = 0;

    // Build frame into a small stack buffer
    uint8_t buf[3 + 250 + 1];
    memcpy(buf, hdr, 3);
    memcpy(buf + 3, payload, len);
    cs = checksum8_frame(buf, 2 + len);   // TYPE+LEN+PAYLOAD length
    buf[3 + len] = cs;

    (void)uart_write_bytes(SERIAL_UART_NUM, (const char *)buf, 3 + len + 1);
}

// ------------- Events / Queue plumbing -------------
typedef enum { APP_EVENT = 0, APP_EVENT_HID_HOST } app_event_group_t;
typedef struct {
    app_event_group_t event_group;
    struct {
        hid_host_device_handle_t handle;
        hid_host_driver_event_t  event;
        void *arg;
    } hid_host_device;
} app_event_queue_t;

static QueueHandle_t app_event_queue = NULL;

static const char *hid_proto_name_str[] = {"NONE", "KEYBOARD", "MOUSE"};

static void hid_host_keyboard_report(const uint8_t *data, size_t len) {
    if (len < sizeof(hid_keyboard_input_report_boot_t)) return;
    uart_send_framed('K', data, sizeof(hid_keyboard_input_report_boot_t));
}

static void hid_host_mouse_report(const uint8_t *data, size_t len) {
    if (len < sizeof(hid_mouse_input_report_boot_t)) return;
    uart_send_framed('M', data, sizeof(hid_mouse_input_report_boot_t));
}

// Interface callback: device is already open & started
static void hid_host_interface_callback(hid_host_device_handle_t dev,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
    (void)arg;
    if (event == HID_HOST_INTERFACE_EVENT_INPUT_REPORT) {
        uint8_t data[64] = {0};
        size_t  data_len = 0;
        hid_host_dev_params_t p;
        ESP_ERROR_CHECK(hid_host_device_get_params(dev, &p));

        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(dev, data, sizeof(data), &data_len));

        if (p.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
            if (p.proto == HID_PROTOCOL_KEYBOARD) {
                hid_host_keyboard_report(data, data_len);
            } else if (p.proto == HID_PROTOCOL_MOUSE) {
                hid_host_mouse_report(data, data_len);
            }
        } else {
            // generic → you could forward as 'R' if desired
            // uart_send_framed('R', data, data_len);
        }
    } else if (event == HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "HID IF DISCONNECTED");
        ESP_ERROR_CHECK(hid_host_device_close(dev));
    }
}

static void hid_host_device_event(hid_host_device_handle_t dev,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
    hid_host_dev_params_t p;
    ESP_ERROR_CHECK(hid_host_device_get_params(dev, &p));

    switch (event) {
        case HID_HOST_DRIVER_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "HID Device CONNECTED (%s)", hid_proto_name_str[p.proto]);
            const hid_host_device_config_t cfg = {
                .callback = hid_host_interface_callback,
                .callback_arg = NULL
            };
            if (p.proto != HID_PROTOCOL_NONE) {
                ESP_ERROR_CHECK(hid_host_device_open(dev, &cfg));
                if (p.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
                    ESP_ERROR_CHECK(hid_class_request_set_protocol(dev, HID_REPORT_PROTOCOL_BOOT));
                    if (p.proto == HID_PROTOCOL_KEYBOARD) {
                        ESP_ERROR_CHECK(hid_class_request_set_idle(dev, 0, 0));
                    }
                }
                ESP_ERROR_CHECK(hid_host_device_start(dev));
            }
        } break;
        default: break;
    }
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {.skip_phy_setup=false, .intr_flags=ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive((TaskHandle_t)arg);

    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

static void gpio_isr_cb(void *arg)
{
    BaseType_t awoke = pdFALSE;
    const app_event_queue_t e = {.event_group = APP_EVENT};
    if (app_event_queue) xQueueSendFromISR(app_event_queue, &e, &awoke);
    if (awoke == pdTRUE) portYIELD_FROM_ISR();
}

static void hid_host_device_callback(hid_host_device_handle_t dev,
                                     const hid_host_driver_event_t event,
                                     void *arg)
{
    const app_event_queue_t e = {
        .event_group = APP_EVENT_HID_HOST,
        .hid_host_device = { .handle=dev, .event=event, .arg=arg }
    };
    if (app_event_queue) xQueueSend(app_event_queue, &e, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "TX: USB HID Host → UART @ %d", SERIAL_BAUD);
    uart_init();

    // BOOT button → shutdown signal
    const gpio_config_t in = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_isr_cb, NULL));

    // Start USB host task
    xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                            xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));
    ESP_LOGI(TAG, "Waiting for HID device... (press BOOT to exit)");

    while (1) {
        app_event_queue_t e;
        if (xQueueReceive(app_event_queue, &e, portMAX_DELAY)) {
            if (e.event_group == APP_EVENT) {
                usb_host_lib_info_t info; ESP_ERROR_CHECK(usb_host_lib_info(&info));
                if (info.num_devices == 0) break;
                ESP_LOGW(TAG, "Remove all USB devices then press BOOT again.");
            } else if (e.event_group == APP_EVENT_HID_HOST) {
                hid_host_device_event(e.hid_host_device.handle,
                                      e.hid_host_device.event,
                                      e.hid_host_device.arg);
            }
        }
    }

    ESP_LOGI(TAG, "Uninstalling HID Host");
    ESP_ERROR_CHECK(hid_host_uninstall());
    gpio_isr_handler_remove(APP_QUIT_PIN);
    vQueueDelete(app_event_queue);
}
