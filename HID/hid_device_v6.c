#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#define UART_PORT      UART_NUM_1
#define UART_RX_PIN    18
#define UART_BUF_SIZE  1024
#define UART_BAUD      115200
#define FRAME_MAGIC    0xA5

static const char *TAG = "hid_receiver";

/******** HID descriptors ********/
const uint8_t hid_kb_report_descriptor[] = { TUD_HID_REPORT_DESC_KEYBOARD() };
const uint8_t hid_mouse_report_descriptor[] = { TUD_HID_REPORT_DESC_MOUSE() };

const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04}, // English
    "TinyUSB", "TinyUSB Device", "123456", "Example HID interface"
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + 2*TUD_HID_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, true, sizeof(hid_kb_report_descriptor), 0x81, 8, 10),
    TUD_HID_DESCRIPTOR(1, 4, false, sizeof(hid_mouse_report_descriptor), 0x82, 8, 10),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return (instance == 0) ? hid_kb_report_descriptor : hid_mouse_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
}

/******** UART frame parsing ********/
typedef struct {
    uint8_t type;
    uint8_t payload_len;
    uint8_t payload[32];
} hid_frame_t;

static bool validate_frame(uint8_t *buf, int len) {
    if(len < 4) return false;
    uint8_t sum = 0;
    for(int i=1; i<len; i++) sum += buf[i];
    return ((sum & 0xFF) == 0);
}

static void uart_rx_task(void *arg) {
    uint8_t buf[UART_BUF_SIZE];
    uint8_t frame[40];
    int frame_len = 0;

    while(1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for(int i=0;i<len;i++) {
            uint8_t b = buf[i];

            // sync on magic
            if(frame_len==0 && b!=FRAME_MAGIC) continue;

            frame[frame_len++] = b;

            if(frame_len>=3) {
                int expected = frame[2]+4;
                if(frame_len==expected) {
                    // validate checksum
                    if(validate_frame(frame, expected)) {
                        ESP_LOGI(TAG, "RX_FRAME: type=%c len=%d payload=", frame[1], frame[2]);
                        for(int j=0;j<frame[2];j++) {
                            printf("%02X ", frame[4+j]);
                        }
                        printf("\n");

                        if(tud_hid_ready()) {
                            if(frame[1]=='K') {
                                uint8_t modifier = frame[3];
                                uint8_t keys[6] = {0};
                                char ascii_keys[6] = {'\0'};
                                int nkeys = frame[2] < 6 ? frame[2] : 6;
                                for(int k=0;k<nkeys;k++) {
                                    keys[k] = frame[4+k];
                                    // Convert to ASCII (A-Z)
                                    if(keys[k] >= 0x04 && keys[k] <= 0x1d) {
                                        ascii_keys[k] = 'A' + keys[k] - 0x04;
                                    } else {
                                        ascii_keys[k] = 0;
                                    }
                                }
                                tud_hid_keyboard_report(0, modifier, keys);
                                ESP_LOGI(TAG, "Keyboard: mod=0x%02X keys=%c %c %c %c %c %c",
                                         modifier,
                                         ascii_keys[0], ascii_keys[1], ascii_keys[2],
                                         ascii_keys[3], ascii_keys[4], ascii_keys[5]);
                            } else if(frame[1]=='M') {
                                uint8_t buttons = frame[3];
                                int8_t dx = (int8_t)frame[4];
                                int8_t dy = (int8_t)frame[5];
                                tud_hid_mouse_report(1, buttons, dx, dy, 0, 0);
                                ESP_LOGI(TAG, "Mouse: buttons=0x%02X dx=%d dy=%d",
                                         buttons, dx, dy);
                            }
                        }
                    } else {
                        ESP_LOGW(TAG, "Checksum invalid, dropping frame");
                    }
                    frame_len = 0;
                } else if(frame_len>=sizeof(frame)) frame_len=0; // safety
            }
        }
    }
}


void app_main(void) {
    ESP_LOGI(TAG,"Starting HID receiver");

    // Init UART
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Start UART RX task
    xTaskCreate(uart_rx_task,"uart_rx_task",4096,NULL,5,NULL);

    // TinyUSB init
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor)/sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if TUD_OPT_HIGH_SPEED
        .fs_configuration_descriptor = hid_configuration_descriptor,
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    while(1) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
