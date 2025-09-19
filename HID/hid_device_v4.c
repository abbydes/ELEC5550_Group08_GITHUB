//Adapted from Espressif Systems ESP-IDF->examples->peripherals->usb->device->tusb_hid
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     921600
#define UART_RX_PIN        18              // Receiver GPIO18
#define UART_BUF_SIZE      1024

static QueueHandle_t uart_queue;           // UART event queue

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

//Define HID report descriptors
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

//Define string descriptors
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

//Defines 1 configuration and 1 HID interface
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

//Print a frame in hex 
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

static bool validate_checksum(uint8_t *frame, int len) {
    if (len < 4) return false;

    print_frame_hex("[RX Frame]", frame, len);

    uint8_t payload_len = frame[2];
    if (len != payload_len + 4) {
        ESP_LOGW("UART_RX", "Frame length mismatch! Expected %d, got %d", payload_len + 4, len);
        return false;
    }

    uint32_t sum = 0;
    for (int i = 1; i < len; i++) {  // skip magic
        sum += frame[i];
    }

    if ((sum & 0xFF) == 0) {
        ESP_LOGI("UART_RX", "Checksum OK");
        return true;
    } else {
        ESP_LOGW("UART_RX", "Checksum FAIL (sum=0x%02X)", sum & 0xFF);
        return false;
    }
}

static void uart_rx_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];
    static uint8_t frame_buf[128];
    static int frame_len = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            uint8_t b = buf[i];

            // --- sync on 0xA5 if buffer empty
            if (frame_len == 0 && b != 0xA5) {
                continue;
            }

            frame_buf[frame_len++] = b;

            // when at least 3 bytes, we know expected length
            if (frame_len >= 3) {
                uint8_t payload_len = frame_buf[2];
                int expected_len = payload_len + 4;

                if (frame_len == expected_len) {
                    // full frame received
                    if (validate_checksum(frame_buf, expected_len)) {
                        if (frame_buf[1] == 'K') {

                        uint8_t modifier = frame_buf[3];
                        uint8_t keycode_array[6] = {0};
                        char ascii_array[6] = {'\0'};

                        // Copy all keycodes from payload (up to 6 keys)
                        int max_keys = (frame_buf[2] < 6) ? frame_buf[2] : 6; // payload length
                        for (int k = 0; k < max_keys; k++) {
                            keycode_array[k] = frame_buf[4 + k]; // payload starts at frame_buf[4]

                            // Convert to ASCII if possible (HID key A-Z = 0x04-0x1D)
                            if (keycode_array[k] >= 0x04 && keycode_array[k] <= 0x1D) {
                                ascii_array[k] = 'A' + keycode_array[k] - 0x04;
                            } else {
                                ascii_array[k] = 0;
                            }
                        }

                        // Send keyboard report with all pressed keys
                        tud_hid_keyboard_report(0, modifier, keycode_array);

                        // Log all pressed keys
                        ESP_LOGI("HID_RX", "Keyboard: mod=0x%02X keys=%c %c %c %c %c %c",
                                modifier,
                                ascii_array[0], ascii_array[1], ascii_array[2],
                                ascii_array[3], ascii_array[4], ascii_array[5]);


                        } else if (frame_buf[1] == 'M') {
                            uint8_t buttons = frame_buf[3];
                            int8_t dx = (int8_t)frame_buf[4];
                            int8_t dy = (int8_t)frame_buf[5];
                            ESP_LOGI("HID_RX", "Mouse: buttons=0x%02X dx=%d dy=%d",
                                    buttons, dx, dy);
                            tud_hid_mouse_report(0, buttons, dx, dy, 0, 0);
                        }
                        // frame good → reset for next frame
                        frame_len = 0;
                    } else {
                        ESP_LOGW("UART_RX", "Checksum FAIL, resyncing...");
                        // frame bad → don’t just reset, try resync
                        // keep only the last byte if it's 0xA5 (potential new header)
                        if (frame_buf[frame_len - 1] == 0xA5) {
                            frame_buf[0] = 0xA5;
                            frame_len = 1;
                        } else {
                            frame_len = 0;
                        }
                    }
                }
                else if (frame_len >= sizeof(frame_buf)) {
                    // safety reset if overflow
                    frame_len = 0;
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

    // UART init section
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
        tud_task();  // keep TinyUSB stack alive
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}
