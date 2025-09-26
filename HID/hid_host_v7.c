// Based on ESP-IDF examples/peripherals/usb/host/hid/main/hid_host_example.c
// ADDITIONS: live prints + UART framing to forward K/M boot reports

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

#define TAG "HID_HOST_LIVE"

// ---- UART bridge to receiver ----
#define UART_NUM_BRIDGE     UART_NUM_1
#define UART_TX_PIN         17
#define UART_BRIDGE_BAUD    115200
#define SERIAL_MAGIC        0xA5

static void uart_bridge_init(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_BRIDGE, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_BRIDGE, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_BRIDGE, UART_TX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void uart_send_frame(uint8_t type, const uint8_t *payload, size_t len) {
    if (!payload || len > 250) return;
    uint8_t buf[3 + 250 + 1];
    buf[0] = SERIAL_MAGIC;
    buf[1] = type;
    buf[2] = (uint8_t)len;
    memcpy(&buf[3], payload, len);
    // checksum so (TYPE+LEN+PAYLOAD+CS) % 256 == 0
    uint32_t s = buf[1] + buf[2];
    for (size_t i = 0; i < len; i++) s += payload[i];
    buf[3 + len] = (uint8_t)((0x100 - (s & 0xFF)) & 0xFF);
    (void)uart_write_bytes(UART_NUM_BRIDGE, (const char*)buf, 3 + len + 1);
}

// ---- Live print helpers ----
static inline bool is_shift(uint8_t mod) {
    return (mod & (HID_LEFT_SHIFT | HID_RIGHT_SHIFT)) != 0;
}

static const uint8_t keycode2ascii[57][2] = {
    {0,0},{0,0},{0,0},{0,0},
    {'a','A'},{'b','B'},{'c','C'},{'d','D'},{'e','E'},{'f','F'},
    {'g','G'},{'h','H'},{'i','I'},{'j','J'},{'k','K'},{'l','L'},
    {'m','M'},{'n','N'},{'o','O'},{'p','P'},{'q','Q'},{'r','R'},
    {'s','S'},{'t','T'},{'u','U'},{'v','V'},{'w','W'},{'x','X'},
    {'y','Y'},{'z','Z'},{'1','!'},{'2','@'},{'3','#'},{'4','$'},
    {'5','%'},{'6','^'},{'7','&'},{'8','*'},{'9','('},{'0',')'},
    {'\r','\r'},{0,0},{'\b',0},{0,0},{' ',' '},{'-','_'},{'=','+'},
    {'[','{'},{']','}'},{'\\','|'},{'\\','|'},{';',':'},{'\'','"'},
    {'`','~'},{',','<'},{'.','>'},{'/','?'}
};

static void kb_live_print(const hid_keyboard_input_report_boot_t *r) {
    uint8_t mod = r->modifier.val;
    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
        uint8_t code = r->key[i];
        if (code >= HID_KEY_A && code <= HID_KEY_SLASH) {
            uint8_t c = keycode2ascii[code][is_shift(mod)];
            if (c) putchar(c);
        } else if (code == HID_KEY_ENTER) {
            putchar('\r'); putchar('\n');
        }
    }
    fflush(stdout);
}

static void mouse_live_print(const hid_mouse_input_report_boot_t *m) {
    printf("Mouse d(%d,%d) buttons:[%c %c %c]\n",
           m->x_displacement, m->y_displacement,
           m->buttons.button1?'L':'-',
           m->buttons.button2?'R':'-',
           m->buttons.button3?'M':'-');
}

// ---- HID host callbacks (same shapes as example) ----
static void hid_host_interface_callback(hid_host_device_handle_t handle,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
    (void)arg;
    if (event == HID_HOST_INTERFACE_EVENT_INPUT_REPORT) {
        uint8_t data[64] = {0};
        size_t  len = 0;
        hid_host_dev_params_t params;
        ESP_ERROR_CHECK(hid_host_device_get_params(handle, &params));
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(handle, data, sizeof(data), &len));

        if (params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
            if (params.proto == HID_PROTOCOL_KEYBOARD && len >= sizeof(hid_keyboard_input_report_boot_t)) {
                const hid_keyboard_input_report_boot_t *kb = (const void*)data;
                kb_live_print(kb);
                uart_send_frame('K', data, sizeof(hid_keyboard_input_report_boot_t));
            } else if (params.proto == HID_PROTOCOL_MOUSE && len >= sizeof(hid_mouse_input_report_boot_t)) {
                const hid_mouse_input_report_boot_t *ms = (const void*)data;
                mouse_live_print(ms);
                uart_send_frame('M', data, sizeof(hid_mouse_input_report_boot_t));
            } else {
                // other boot-type length; ignore or forward as raw
            }
        } else {
            // Non-boot reports → optionally forward as raw 'R'
            // uart_send_frame('R', data, len);
        }
    } else if (event == HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Interface disconnected");
        ESP_ERROR_CHECK(hid_host_device_close(handle));
    }
}

static void hid_host_device_event(hid_host_device_handle_t handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
    (void)arg;
    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        hid_host_dev_params_t p; ESP_ERROR_CHECK(hid_host_device_get_params(handle, &p));
        ESP_LOGI(TAG, "Device CONNECTED proto=%d", p.proto);
        const hid_host_device_config_t cfg = {
            .callback = hid_host_interface_callback,
            .callback_arg = NULL
        };
        if (p.proto != HID_PROTOCOL_NONE) {
            ESP_ERROR_CHECK(hid_host_device_open(handle, &cfg));
            if (p.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
                ESP_ERROR_CHECK(hid_class_request_set_protocol(handle, HID_REPORT_PROTOCOL_BOOT));
                if (p.proto == HID_PROTOCOL_KEYBOARD) {
                    ESP_ERROR_CHECK(hid_class_request_set_idle(handle, 0, 0));
                }
            }
            ESP_ERROR_CHECK(hid_host_device_start(handle));
        }
    }
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t cfg = {.skip_phy_setup=false, .intr_flags=ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&cfg));
    xTaskNotifyGive((TaskHandle_t)arg);

    while (1) {
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

void app_main(void)
{
    ESP_LOGI(TAG, "Host live demo + UART bridge");
    uart_bridge_init();

    // launch lib task
    xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                            xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_event,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    // Just idle; callbacks do the work
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
