/*
---------------- ELEC5550 - TEAM_08 - hid_device_v9 ----------------
Final HID device software 
*/

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     8000000
#define UART_RX_PIN        18
#define UART_BUF_SIZE      1024

static QueueHandle_t uart_queue;

#define APP_BUTTON (GPIO_NUM_0)
static const char *TAG = "HID_DEVICE";

/************* TinyUSB descriptors ****************/

const uint8_t hid_kb_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

const uint8_t hid_mouse_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},
    "TinyUSB",
    "TinyUSB Composite Device",
    "123456",
    "HID Keyboard/Mouse",
};

#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN)

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, HID_ITF_PROTOCOL_KEYBOARD, sizeof(hid_kb_report_descriptor), 0x81, 16, 10),
    TUD_HID_DESCRIPTOR(1, 4, HID_ITF_PROTOCOL_MOUSE, sizeof(hid_mouse_report_descriptor), 0x82, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    ESP_LOGI(TAG, "Host requesting report descriptor for instance %d", instance);
    if (instance == 0) {
        return hid_kb_report_descriptor;
    } else if (instance == 1) {
        return hid_mouse_report_descriptor;
    }
    return NULL;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    ESP_LOGI(TAG, "GET_REPORT: instance=%d, report_id=%d", instance, report_id);
    (void) instance; (void) report_id; (void) report_type;
    (void) buffer; (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    ESP_LOGI(TAG, "SET_REPORT: instance=%d, report_id=%d", instance, report_id);
}

// TinyUSB mount/unmount callbacks
void tud_mount_cb(void)
{
    ESP_LOGI(TAG, "USB DEVICE MOUNTED - Host recognized device!");
}

void tud_umount_cb(void)
{
    ESP_LOGW(TAG, "USB DEVICE UNMOUNTED");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    ESP_LOGW(TAG, "USB SUSPENDED");
}

void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB RESUMED");
}

/********* Application ***************/

static void print_frame_hex(const char *tag, uint8_t *frame, int len) {
    printf("%s ", tag);
    for (int i = 0; i < len; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");
}

static bool validate_checksum(uint8_t *frame, int len) {
    if (len < 4) return false;

    print_frame_hex("[RX Frame]", frame, len);

    uint8_t payload_len = frame[2];
    if (len != payload_len + 4) {
        ESP_LOGW(TAG, "Frame length mismatch! Expected %d, got %d", payload_len + 4, len);
        return false;
    }

    uint32_t sum = 0;
    for (int i = 1; i < len; i++) {
        sum += frame[i];
    }

    if ((sum & 0xFF) == 0) {
        return true;
    } else {
        ESP_LOGW(TAG, "Checksum FAIL (sum=0x%02X)", sum & 0xFF);
        return false;
    }
}

static void uart_rx_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];
    static uint8_t frame_buf[128];
    static int frame_len = 0;
    static uint32_t mouse_report_count = 0;
    static uint32_t kb_report_count = 0;

    ESP_LOGI(TAG, "UART RX Task started");

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        
        for (int i = 0; i < len; i++) {
            uint8_t b = buf[i];

            if (frame_len == 0 && b != 0xA5) {
                continue;
            }

            frame_buf[frame_len++] = b;

            if (frame_len >= 3) {
                uint8_t payload_len = frame_buf[2];
                int expected_len = payload_len + 4;

                if (frame_len == expected_len) {
                    if (validate_checksum(frame_buf, expected_len)) {
                        
                        if (frame_buf[1] == 'K') {
                            kb_report_count++;
                            uint8_t modifier = frame_buf[3];
                            uint8_t keycode_array[6] = {0};

                            for (int k = 0; k < 6; k++) {
                                keycode_array[k] = frame_buf[5 + k];
                            }

                            bool mounted = tud_mounted();
                            bool ready = tud_hid_n_ready(0);
                            
                            ESP_LOGI(TAG, "KB Report #%lu: mounted=%d ready=%d mod=0x%02X", 
                                     kb_report_count, mounted, ready, modifier);

                            if (mounted && ready) {
                                bool sent = tud_hid_n_keyboard_report(0, 0, modifier, keycode_array);
                                ESP_LOGI(TAG, "  -> Keyboard report sent: %s", sent ? "OK" : "FAIL");
                            } else {
                                ESP_LOGW(TAG, "  -> Skipped (not ready)");
                            }

                        } else if (frame_buf[1] == 'M') {
                            mouse_report_count++;
                            uint8_t buttons = frame_buf[3];
                            int8_t dx = (int8_t)frame_buf[4];
                            int8_t dy = (int8_t)frame_buf[5];
                            int8_t wheel = (frame_buf[2] >= 4) ? (int8_t)frame_buf[6] : 0;
                            
                            bool mounted = tud_mounted();
                            bool ready = tud_hid_n_ready(1);
                            
                            ESP_LOGI(TAG, "MOUSE Report #%lu: mounted=%d ready=%d btn=0x%02X dx=%d dy=%d", 
                                     mouse_report_count, mounted, ready, buttons, dx, dy);
                            
                            if (mounted && ready) {
                                bool sent = tud_hid_n_mouse_report(1, 0, buttons, dx, dy, wheel, 0);
                                ESP_LOGI(TAG, "  -> Mouse report sent: %s", sent ? "OK" : "FAIL");
                                
                                // Add small delay to prevent overwhelming the USB
                                vTaskDelay(pdMS_TO_TICKS(1));
                            } else {
                                ESP_LOGW(TAG, "  -> Skipped (not ready)");
                            }
                        }
                        frame_len = 0;
                    } else {
                        ESP_LOGW(TAG, "Checksum FAIL, dropping frame");
                        frame_len = 0;
                    }
                }
                else if (frame_len >= sizeof(frame_buf)) {
                    ESP_LOGE(TAG, "Frame buffer overflow! Resetting.");
                    frame_len = 0;
                }
            }
        }
        
        // Status report every 5 seconds
        static uint32_t last_status = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_status > 5000) {
            ESP_LOGI(TAG, "Status: Mounted=%d, KB_reports=%lu, Mouse_reports=%lu", 
                     tud_mounted(), kb_report_count, mouse_report_count);
            last_status = now;
        }
    }
}

void app_main(void)
{
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    ESP_LOGI(TAG, "=== USB HID Composite Device (Keyboard + Mouse) ===");
    ESP_LOGI(TAG, "Initializing TinyUSB...");
    
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor,
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB driver installed");

    // UART init
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                             UART_PIN_NO_CHANGE,
                             UART_RX_PIN,
                             UART_PIN_NO_CHANGE,
                             UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART initialized on GPIO%d @ %d baud", UART_RX_PIN, UART_BAUD_RATE);

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART RX task created");

    ESP_LOGI(TAG, "=== Waiting for USB enumeration ===");
    ESP_LOGI(TAG, "Connect this ESP32-S3 to your MacBook now...");

    while (1) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
