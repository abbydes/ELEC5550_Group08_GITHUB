// Based on ESP-IDF examples/peripherals/usb/device/tusb_hid/main/tusb_hid_example_main.c
// CHANGES: two HID interfaces (keyboard+mouse) + UART listener that injects reports

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#define TAG "HID_DEV_BRIDGE"

// ---- UART from host board ----
#define UART_PORT_NUM      UART_NUM_1
#define UART_RX_PIN        18
#define UART_BAUD          115200
#define UART_BUF_SZ        512
#define SERIAL_MAGIC       0xA5

// ---- TinyUSB descriptors (2 HID IFs) ----
static const uint8_t desc_kb[]   = { TUD_HID_REPORT_DESC_KEYBOARD() };
static const uint8_t desc_mouse[] = { TUD_HID_REPORT_DESC_MOUSE() };

static const char *str_desc[] = {
    (const char[]){ 0x09, 0x04 }, // 0: English
    "Espressif",                  // 1: Manufacturer
    "ESP32-S3 HID Bridge",        // 2: Product
    "123456",                     // 3: Serial
    "HID IF",                     // 4: Interface string
};

#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2*TUD_HID_DESC_LEN)

static const uint8_t cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // ifc, str, boot, rep_len, ep_in, size, interval
    TUD_HID_DESCRIPTOR(0, 4, true,  sizeof(desc_kb),    0x81, 8, 10), // KBD
    TUD_HID_DESCRIPTOR(1, 4, true,  sizeof(desc_mouse), 0x82, 8, 2),  // MOUSE
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
    if (itf == 0) return desc_kb;
    if (itf == 1) return desc_mouse;
    return NULL;
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t id, hid_report_type_t type,
                               uint8_t *buf, uint16_t reqlen) {
    (void)itf; (void)id; (void)type; (void)buf; (void)reqlen;
    return 0;
}
void tud_hid_set_report_cb(uint8_t itf, uint8_t id, hid_report_type_t type,
                           uint8_t const *buf, uint16_t len) {
    (void)itf; (void)id; (void)type; (void)buf; (void)len;
}

// ---- UART framing → HID injection ----
static inline bool frame_ok(const uint8_t *f, int n) {
    if (n < 4) return false;
    uint8_t plen = f[2];
    if (plen + 4 != n) return false;
    uint32_t s = 0;
    for (int i = 1; i < n; i++) s += f[i];
    return ((s & 0xFF) == 0);
}

static void uart_task(void *arg) {
    (void)arg;
    uint8_t rx[UART_BUF_SZ];
    uint8_t f[128]; int flen = 0;

    while (1) {
        int n = uart_read_bytes(UART_PORT_NUM, rx, sizeof(rx), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            uint8_t b = rx[i];
            if (flen == 0 && b != SERIAL_MAGIC) continue; // sync
            f[flen++] = b;

            if (flen >= 3) {
                int expect = f[2] + 4;
                if (flen == expect) {
                    if (frame_ok(f, flen)) {
                        uint8_t type = f[1];
                        uint8_t *p = &f[3];
                        uint8_t plen = f[2];
                        if (type == 'K' && plen >= 8) {
                            // Boot keyboard: [mod(1), reserved(1), keys[6]]
                            uint8_t mod = p[0];
                            uint8_t key6[6];
                            memcpy(key6, p + 2, 6);
                            if (tud_hid_n_ready(0)) {
                                tud_hid_n_keyboard_report(0, 0, mod, key6);
                            }
                        } else if (type == 'M' && plen >= 3) {
                            // Boot mouse: [buttons, dx, dy, (wheel), (pan)]
                            uint8_t buttons = p[0];
                            int8_t dx = (int8_t)p[1];
                            int8_t dy = (int8_t)p[2];
                            int8_t wheel = (plen >= 4) ? (int8_t)p[3] : 0;
                            int8_t pan   = (plen >= 5) ? (int8_t)p[4] : 0;
                            if (tud_hid_n_ready(1)) {
                                tud_hid_n_mouse_report(1, 0, buttons, dx, dy, wheel, pan);
                            }
                        }
                    }
                    flen = 0;
                } else if (flen >= (int)sizeof(f)) {
                    flen = 0;
                }
            }
        }
    }
}

void app_main(void)
{
    // TinyUSB init (matches example structure)
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = str_desc,
        .string_descriptor_count = sizeof(str_desc)/sizeof(str_desc[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = cfg_desc,
        .hs_configuration_descriptor = cfg_desc,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = cfg_desc,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB device ready: KBD itf0 + MOUSE itf1");

    // UART init
    uart_config_t uc = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SZ, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uc));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);

    // If your port doesn’t require tud_task() here, this loop can idle
    while (1) vTaskDelay(pdMS_TO_TICKS(100));
}
