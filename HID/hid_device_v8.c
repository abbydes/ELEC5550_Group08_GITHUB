//Adapted from Espressif Systems ESP-IDF->examples->peripherals->usb->device->tusb_hid

//--------FLASH CODE VIA USB-UART PORT THEN UNPLUG AND CONNECT VIA USB-OTG PORT FOR TESTING--------
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_RX_PIN        18              // Receiver GPIO18
#define UART_BUF_SIZE      1024

static QueueHandle_t uart_queue;           // UART event queue

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "hid_receiver";

/************* TinyUSB descriptors ****************/

// Keyboard report descriptor (standard boot keyboard)
const uint8_t hid_kb_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD() // macro from tinyusb
};

// Mouse report descriptor (standard boot mouse)
const uint8_t hid_mouse_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE() // macro from tinyusb
};

//Define string descriptors
const char *hid_string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

// total length: config desc + 2 * HID desc length
#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN)

// Configuration descriptor with TWO HID interfaces (keyboard = itf 0, mouse = itf 1)
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration: 1 config, 2 interfaces
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // HID Interface 0 (Keyboard)
    // ifc_number, string_index, protocol, report_desc_len, EPin, size, interval
    TUD_HID_DESCRIPTOR(0, 4, true, sizeof(hid_kb_report_descriptor), 0x81, 8, 10),

    // HID Interface 1 (Mouse)
    TUD_HID_DESCRIPTOR(1, 4, false, sizeof(hid_mouse_report_descriptor), 0x82, 8, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Return the correct HID report descriptor for the requested interface instance
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    if (instance == 0) {
        // keyboard interface
        return hid_kb_report_descriptor;
    } else if (instance == 1) {
        // mouse interface
        return hid_mouse_report_descriptor;
    }
    return NULL;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void) instance; (void) report_id; (void) report_type;
    (void) buffer; (void) reqlen;
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void) instance; (void) report_id; (void) report_type;
    (void) buffer; (void) bufsize;
}

/********* Application ***************/

static void print_frame_hex(const char *tag, uint8_t *frame, int len)
{
    ESP_LOGI(TAG, "%s:", tag);
    char bfr[256];
    int pos = 0;
    for (int i = 0; i < len && pos < (int)sizeof(bfr)-4; i++) {
        pos += snprintf(bfr + pos, sizeof(bfr) - pos, "%02X ", frame[i]);
    }
    bfr[pos] = 0;
    ESP_LOGI(TAG, "%s", bfr);
}

static bool validate_checksum(uint8_t *frame, int len) {
    if (len < 4) return false;

    uint8_t payload_len = frame[2];
    if (len != payload_len + 4) {
        ESP_LOGW("UART_RX", "Frame length mismatch! Expected %d, got %d", payload_len + 4, len);
        return false;
    }

    print_frame_hex("[RX Frame]", frame, len);

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
    (void) arg;
    uint8_t buf[UART_BUF_SIZE];
    static uint8_t frame_buf[256];
    int frame_len = 0;

    // track last state for release detection / debounce
    static uint8_t last_modifier = 0;
    static uint8_t last_keys[6] = {0};
    static uint8_t last_buttons = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (len <= 0) {
            // allow TinyUSB tasks to run if needed
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        for (int i = 0; i < len; i++) {
            uint8_t b = buf[i];

            // if buffer empty, sync to magic 0xA5
            if (frame_len == 0) {
                if (b != 0xA5) {
                    continue; // skip until magic found
                }
            }

            // append
            if (frame_len < (int)sizeof(frame_buf)) {
                frame_buf[frame_len++] = b;
            } else {
                // overflow: reset and wait for next header
                ESP_LOGW(TAG, "Frame buffer overflow, resync");
                frame_len = 0;
                continue;
            }

            // as soon as we have 3 bytes we know payload length
            if (frame_len >= 3) {
                uint8_t payload_len = frame_buf[2];
                int expected_len = payload_len + 4; // magic(1)+type(1)+len(1)+payload(len)+cs(1)

                if (frame_len == expected_len) {
                    // full frame
                    if (validate_checksum(frame_buf, expected_len)) {
                        uint8_t pkt_type = frame_buf[1];
                        uint8_t *payload = &frame_buf[3];

                        if (pkt_type == 'K') {
                            // Expecting boot keyboard report payload: [0]=modifier, [1]=reserved, [2..7]=6 key codes
                            // but transmitter used sizeof(hid_keyboard_input_report_boot_t) likely 8 bytes.
                            uint8_t modifier = payload[0];
                            // keys start at payload[2]
                            uint8_t keys[6] = {0};
                            for (int k = 0; k < 6; k++) keys[k] = payload[2 + k];

                            // send keyboard report if changed (press or release)
                            if (tud_hid_ready()) {
                                bool changed = (modifier != last_modifier) || (memcmp(keys, last_keys, 6) != 0);
                                if (changed) {
                                    // forward as boot keyboard report (report id 0)
                                    tud_hid_keyboard_report(0, modifier, keys);
                                    last_modifier = modifier;
                                    memcpy(last_keys, keys, 6);
                                    ESP_LOGI(TAG, "Forward keyboard: mod=0x%02X keys=%02X %02X %02X %02X %02X %02X",
                                             modifier,
                                             keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);
                                }
                            } else {
                                ESP_LOGW(TAG, "tud_hid not ready, keyboard frame dropped");
                            }
                        }
                        else if (pkt_type == 'M') {
                            // Expecting mouse boot report payload: [0]=buttons, [1]=x, [2]=y  (signed)
                            // But transmitter used sizeof(hid_mouse_input_report_boot_t) typically 3 bytes.
                            uint8_t buttons = payload[0];
                            int8_t dx = (int8_t)payload[1];
                            int8_t dy = (int8_t)payload[2];

                            if (tud_hid_ready()) {
                                // Always send a single report containing buttons + dx/dy
                                // (this avoids inconsistent states where movement and buttons become unsynced)
                                tud_hid_mouse_report(0 /*report id*/, buttons, dx, dy, 0, 0);
                                last_buttons = buttons;
                                ESP_LOGI(TAG, "Forward mouse: buttons=0x%02X dx=%d dy=%d", buttons, dx, dy);
                            } else {
                                ESP_LOGW(TAG, "tud_hid not ready, mouse frame dropped");
                            }
                        } else {
                            ESP_LOGW(TAG, "Unknown frame type '%c' (0x%02X)", frame_buf[1], frame_buf[1]);
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid checksum -> drop frame and resync");
                    }

                    // reset for next frame
                    frame_len = 0;
                } else if (frame_len > expected_len) {
                    // Shouldn't happen but handle: drop and resync
                    ESP_LOGW(TAG, "frame_len > expected_len -- resync");
                    frame_len = 0;
                }
            }
        }
    }
}

void app_main(void)
{
    // Configure BOOT button (not used heavily but keep as example)
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    ESP_LOGI(TAG, "UART init");
    // UART init section
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // install uart driver (RX only)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                             UART_PIN_NO_CHANGE,   // TX not used
                             UART_RX_PIN,          // RX pin = GPIO18
                             UART_PIN_NO_CHANGE,   // RTS
                             UART_PIN_NO_CHANGE)); // CTS

    // TinyUSB init
    ESP_LOGI(TAG, "TinyUSB init");
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
    ESP_LOGI(TAG, "TinyUSB installed");

    // create UART reader task
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    // keep TinyUSB stack alive in main loop
    while (1) {
        tud_task();  // tinyusb device task
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}