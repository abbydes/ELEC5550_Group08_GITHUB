// Receiver: ESP32-S3 enumerates as USB HID (Keyboard + Mouse).
// It reads framed UART packets from the TX and injects HID reports to the host PC.

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#define TAG "RX"

// ---------------- UART from transmitter ----------------
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200        // match TX
#define UART_RX_PIN        18
#define UART_BUF_SIZE      512
#define SERIAL_MAGIC       0xA5
// -------------------------------------------------------

// BOOT button just reserved (not used)
#define APP_BUTTON         GPIO_NUM_0

/************* TinyUSB descriptors ****************/

// Standard boot descriptors
static const uint8_t hid_kb_report_descriptor[]   = { TUD_HID_REPORT_DESC_KEYBOARD() };
static const uint8_t hid_mouse_report_descriptor[] = { TUD_HID_REPORT_DESC_MOUSE() };

static const char *hid_string_descriptor[] = {
    (const char[]){ 0x09, 0x04 }, // 0: English (0x0409)
    "TinyUSB",                    // 1: Manufacturer
    "TinyUSB Composite HID",      // 2: Product
    "123456",                     // 3: Serial
    "HID Interface"               // 4: String for both interfaces
};

// Total length: config + 2*HID interface descriptors
#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN)

static const uint8_t hid_configuration_descriptor[] = {
    // Config: 1 cfg, 2 interfaces, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // HID Interface 0: Keyboard (BOOT protocol)
    // itf, stridx, boot, report_len, epin, size, interval(ms)
    TUD_HID_DESCRIPTOR(0, 4, true,  sizeof(hid_kb_report_descriptor),   0x81, 8, 10),

    // HID Interface 1: Mouse (BOOT protocol)
    TUD_HID_DESCRIPTOR(1, 4, true,  sizeof(hid_mouse_report_descriptor), 0x82, 8, 2),
};

// TinyUSB multi-instance helpers
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    if (instance == 0) return hid_kb_report_descriptor;
    if (instance == 1) return hid_mouse_report_descriptor;
    return NULL;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0; // no feature/input report on request
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}

/************* UART framing + HID injection **************/

static inline bool frame_checksum_ok(const uint8_t *f, int len) {
    if (len < 4) return false;
    uint8_t plen = f[2];
    if (plen + 4 != len) return false;

    uint32_t s = 0;
    for (int i = 1; i < len; i++) s += f[i]; // TYPE+LEN+PAYLOAD+CS
    return ((s & 0xFF) == 0);
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t rx[UART_BUF_SIZE];
    uint8_t f[128]; int flen = 0;

    while (1) {
        int n = uart_read_bytes(UART_PORT_NUM, rx, sizeof(rx), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            uint8_t b = rx[i];

            if (flen == 0 && b != SERIAL_MAGIC) continue;   // sync
            f[flen++] = b;

            if (flen >= 3) {
                int expect = f[2] + 4;
                if (flen == expect) {
                    // Full frame
                    if (frame_checksum_ok(f, flen)) {
                        uint8_t type = f[1];
                        uint8_t *payload = &f[3];
                        uint8_t plen = f[2];

                        if (type == 'K' && plen >= 8) {
                            // Boot keyboard report: [mod(1), reserved(1), keys[6]]
                            uint8_t mod = payload[0];
                            uint8_t key6[6];
                            memcpy(key6, payload + 2, 6);

                            if (tud_hid_n_ready(0)) {
                                // instance 0 = keyboard
                                tud_hid_n_keyboard_report(0, 0 /*report_id*/,
                                                          mod, key6);
                            }

                        } else if (type == 'M' && plen >= 3) {
                            // Boot mouse report: [buttons(1), dx(1), dy(1)] (wheel/pan optional)
                            uint8_t buttons = payload[0];
                            int8_t  dx = (int8_t)payload[1];
                            int8_t  dy = (int8_t)payload[2];
                            int8_t  wheel = (plen >= 4) ? (int8_t)payload[3] : 0;
                            int8_t  pan   = (plen >= 5) ? (int8_t)payload[4] : 0;

                            if (tud_hid_n_ready(1)) {
                                // instance 1 = mouse
                                tud_hid_n_mouse_report(1, 0 /*report_id*/,
                                                       buttons, dx, dy, wheel, pan);
                            }
                        }
                    } else {
                        ESP_LOGW(TAG, "Bad checksum, drop");
                    }
                    flen = 0; // reset
                } else if (flen >= (int)sizeof(f)) {
                    flen = 0; // overflow safety
                }
            }
        }
    }
}

void app_main(void)
{
    // Button (not used, but leave configured)
    const gpio_config_t btn = {
        .pin_bit_mask = BIT64(APP_BUTTON), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn);

    // UART init
    const uart_config_t uc = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uc));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                                 UART_PIN_NO_CHANGE,    // TX not used
                                 UART_RX_PIN,           // RX=GPIO18
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    // TinyUSB init
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
    ESP_LOGI(TAG, "USB HID ready (KBD itf=0, MOUSE itf=1)");

    // RX task
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    // If your TinyUSB port runs its own task, this loop can be idle.
    while (1) vTaskDelay(pdMS_TO_TICKS(100));
}
