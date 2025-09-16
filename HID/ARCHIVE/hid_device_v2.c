/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/uart.h"


#define UART_PORT_NUM      UART_NUM_1      // Use UART1
#define UART_BAUD_RATE     115200
#define UART_RX_PIN        18              // Match sender TX, supposed to be GPIO18
#define UART_BUF_SIZE      1024

static QueueHandle_t uart_queue;           // <--- NEW: UART event queue

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
}

/********* Application ***************/

// <--- NEW: helper to print a frame in hex
static void print_frame_hex(const char *tag, uint8_t *frame, int len) {
    printf("%s ", tag);
    for (int i = 0; i < len; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");
}

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX        125
#define DELTA_SCALAR        5

// <--- NEW checksum helper
static bool validate_checksum(uint8_t *frame, int len) {
    if (len < 4) return false; // too short

    //print the raw frame
    print_frame_hex("[RX Frame]", frame, len);

    uint8_t type = frame[1];
    uint8_t payload_len = frame[2];

    //make sure payload length in header matches received data and frame is correct
    if (len != payload_len + 4) {
        ESP_LOGW("UART_RX", "Frame length mismatch! Expected %d, got %d", payload_len + 4, len);
        return false;
    }

    uint32_t sum = type + payload_len;
    for (int i = 0; i < payload_len; i++) {
        sum += frame[3 + i]; // payload starts at frame[3]
    }

    uint8_t expected = (uint8_t)(0x100 - (sum & 0xFF));
    uint8_t received = frame[len - 1];

    if (received == expected) {
        ESP_LOGI("UART_RX", "Checksum OK (0x%02X)", received);
        return true;
    } else {
        ESP_LOGW("UART_RX", "Checksum FAIL: got 0x%02X, expected 0x%02X", received, expected);
        return false;
    }
}


static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX) {
            cur_dir = 0;
        }
    }
}

static void app_send_hid_demo(void)
{
    // Keyboard output: Send key 'a/A' pressed and released
    ESP_LOGI(TAG, "Sending Keyboard report");
    uint8_t keycode[6] = {HID_KEY_A};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    vTaskDelay(pdMS_TO_TICKS(50));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void uart_rx_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    char log_buf[128];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            // Only process frames starting with 0xA5
            if (data[0] == 0xA5) {
                if (validate_checksum(data, len)) {
                    // Keyboard frame
                    if (data[1] == 'K') {
                        uint8_t modifier = data[3];
                        uint8_t keycode  = data[5];
                        char ascii = '?';
                        if (keycode >= 0x04 && keycode <= 0x1D) {
                            ascii = 'A' + keycode - 0x04;
                        }

                        // Log human-readable data
                        snprintf(log_buf, sizeof(log_buf),
                                 "Keyboard Received: Modifier=0x%02X, Keycode=0x%02X ('%c')\n",
                                 modifier, keycode, ascii);
                        uart_write_bytes(UART_PORT_NUM, log_buf, strlen(log_buf));

                        // ALSO print to console monitor
                        ESP_LOGI("HID_RX", "%s", log_buf);

                        // Inject HID report
                        uint8_t keycode_array[6] = {0};
                        keycode_array[0] = keycode;
                        tud_hid_keyboard_report(0, modifier, keycode_array);
                    }
                    // Mouse frame
                    else if (data[1] == 'M') {
                        uint8_t buttons = data[2];
                        int8_t dx = (int8_t)data[3];
                        int8_t dy = (int8_t)data[4];

                        // Human-readable buttons
                        char buttons_str[32] = "";
                        if (buttons & 0x01) strcat(buttons_str, "Left ");
                        if (buttons & 0x02) strcat(buttons_str, "Right ");
                        if (buttons & 0x04) strcat(buttons_str, "Middle ");

                        // Log human-readable data
                        snprintf(log_buf, sizeof(log_buf),
                                 "Mouse Received: Buttons=%s, dx=%d, dy=%d\n",
                                 buttons_str, dx, dy);
                        uart_write_bytes(UART_PORT_NUM, log_buf, strlen(log_buf));

                        // ALSO print to console monitor
                        ESP_LOGI("HID_RX", "%s", log_buf);

                        // Inject HID report
                        tud_hid_mouse_report(0, buttons, dx, dy, 0, 0);
                    }
                } else {
                    ESP_LOGW("UART_RX", "Discarded invalid frame");
                }
            }
        }
    }
}

void app_main(void)
{
    // Initialize button that will trigger HID reports
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor, // HID configuration descriptor for full-speed and high-speed are the same
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif // TUD_OPT_HIGH_SPEED
    };

    // <--- NEW UART init section
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                             UART_PIN_NO_CHANGE,   // TX not used
                             UART_RX_PIN,          // RX pin = GPIO18
                             UART_PIN_NO_CHANGE,   // RTS
                             UART_PIN_NO_CHANGE)); // CTS

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) {
        if (tud_mounted()) {
            static bool send_hid_data = true;
            if (send_hid_data) {
                app_send_hid_demo();
            }
            send_hid_data = !gpio_get_level(APP_BUTTON);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
