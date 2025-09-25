// boardA_msc_bridge.c
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tusb_msc_storage.h"
#include "descriptors_control.h"
#include "usb_descriptors.h"

 
#include "tinyusb.h"
#include "class/msc/msc_device.h"
 
static const char *TAG = "boardA_msc_bridge";
 
// ---------------- USB MSC descriptors ----------------
#define EPNUM_MSC_OUT   0x01
#define EPNUM_MSC_IN    0x81
#define MSC_EPSIZE      64
 
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MSC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCafe,
    .idProduct = 0x4000,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

/*
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
} */
 
static const char *string_desc[] = {
    (const char[]){0x09, 0x04}, // English language
    "BoardA Manufacturer",      // Manufacturer name
    "BoardA MSC",               // Product name
    "0001",                     // Serial number
    // "Example MSC",           // MSC interface name
};

/*
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    static uint16_t desc_str[32];
    if (index == 0) {
        desc_str[0] = 0x0300 | (1 * 2 + 2);
        desc_str[1] = 0x0409;
        return desc_str;
    }
    if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
    const char *str = string_desc_arr[index];
    uint8_t chr_count = 0;
    for (; chr_count < 31 && str[chr_count]; chr_count++) {
        desc_str[1+chr_count] = str[chr_count];
    }
    desc_str[0] = 0x0300 | (2*chr_count + 2);
    return desc_str;
} */

// Configuration descriptors
enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

uint8_t const msc_fs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 4, EPNUM_MSC_OUT, EPNUM_MSC_IN, MSC_EPSIZE)
};

//tusb includes

// Called during SCSI INQUIRY command
void tud_msc_inquiry_cb(uint8_t lun,
                        uint8_t vendor_id[8],
                        uint8_t product_id[16],
                        uint8_t product_rev[4]) {
    const char vid[] = "BoardA";
    const char pid[] = "MSCBridge";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memset(vendor_id + strlen(vid), ' ', 8 - strlen(vid));

    memcpy(product_id, pid, strlen(pid));
    memset(product_id + strlen(pid), ' ', 16 - strlen(pid));

    memcpy(product_rev, rev, strlen(rev));
    memset(product_rev + strlen(rev), ' ', 4 - strlen(rev));
}

// Called for SCSI commands not built in
int32_t tud_msc_scsi_cb(uint8_t lun,
                        uint8_t const scsi_cmd[16],
                        void* buffer,
                        uint16_t bufsize) {
    (void) lun; (void) buffer; (void) bufsize;

    // Unsupported → fail
    return -1;
}

                       
// ---------------- UART CONFIG ----------------
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX_PIN     18
#define UART_RX_PIN     17
#define UART_BAUD_RATE  115200
#define UART_BUF_SIZE   4096
 
// ---------------- MSC REQUEST QUEUE ----------------
typedef enum { REQ_READ10, REQ_WRITE10 } msc_req_type_t;
 
typedef struct {
    msc_req_type_t type;
    uint32_t lba;
    uint32_t bufsize;
    void *buffer;
} msc_req_t;
 
static QueueHandle_t msc_queue;
 
// ---------------- MSC DEVICE CALLBACKS ----------------
#define SECTOR_SIZE   512
#define DISK_BLOCK_NUM 65536
 
bool tud_msc_test_unit_ready_cb(uint8_t lun) { return true; }
 
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    *block_count = DISK_BLOCK_NUM;
    *block_size  = SECTOR_SIZE;
}
 
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) { return true; }
 
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    msc_req_t req = {.type = REQ_READ10, .lba = lba, .bufsize = bufsize, .buffer = buffer};
    if (xQueueSend(msc_queue, &req, 0) != pdTRUE) return -1;
    return bufsize;
}
 
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    msc_req_t req = {.type = REQ_WRITE10, .lba = lba, .bufsize = bufsize, .buffer = buffer};
    if (xQueueSend(msc_queue, &req, 0) != pdTRUE) return -1;
    return bufsize;
}
 
// ---------------- MSC WORKER TASK ----------------
static void msc_worker_task(void *arg) {
    msc_req_t req;
    uint8_t ack;
    while (1) {
        if (xQueueReceive(msc_queue, &req, portMAX_DELAY)) {
            uint8_t frame[9];
            frame[1] = (req.lba >> 16) & 0xFF; // just placeholder
            if (req.type == REQ_READ10) {
                frame[0] = 'R';
                frame[1] = (req.lba >> 24) & 0xFF;
                frame[2] = (req.lba >> 16) & 0xFF;
                frame[3] = (req.lba >>  8) & 0xFF;
                frame[4] = (req.lba >>  0) & 0xFF;
                frame[5] = (req.bufsize >> 24) & 0xFF;
                frame[6] = (req.bufsize >> 16) & 0xFF;
                frame[7] = (req.bufsize >>  8) & 0xFF;
                frame[8] = (req.bufsize >>  0) & 0xFF;
 
                uart_write_bytes(UART_PORT_NUM, (char*)frame, sizeof(frame));
                int rx = uart_read_bytes(UART_PORT_NUM, req.buffer, req.bufsize, pdMS_TO_TICKS(2000));
                if (rx != req.bufsize) ESP_LOGE(TAG,"Short read %d/%d", rx, req.bufsize);
            } else if (req.type == REQ_WRITE10) {
                frame[0] = 'W';
                frame[1] = (req.lba >> 24) & 0xFF;
                frame[2] = (req.lba >> 16) & 0xFF;
                frame[3] = (req.lba >>  8) & 0xFF;
                frame[4] = (req.lba >>  0) & 0xFF;
                frame[5] = (req.bufsize >> 24) & 0xFF;
                frame[6] = (req.bufsize >> 16) & 0xFF;
                frame[7] = (req.bufsize >>  8) & 0xFF;
                frame[8] = (req.bufsize >>  0) & 0xFF;
 
                uart_write_bytes(UART_PORT_NUM, (char*)frame, sizeof(frame));
                uart_write_bytes(UART_PORT_NUM, req.buffer, req.bufsize);
 
                int rx = uart_read_bytes(UART_PORT_NUM, &ack, 1, pdMS_TO_TICKS(1000));
                if (rx != 1 || ack != 0xAA) ESP_LOGE(TAG,"Write ack failed");
            }
        }
    }
}
 
// ---------------- MAIN ----------------
void app_main(void) {
    ESP_LOGI(TAG,"Board A MSC bridge starting...");
 
    // MSC request queue
    msc_queue = xQueueCreate(8, sizeof(msc_req_t));
    if (!msc_queue) { ESP_LOGE(TAG,"Queue creation failed"); return; }
 
    xTaskCreate(msc_worker_task,"msc_worker",4096,NULL,5,NULL);
 
    // Init TinyUSB device
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .configuration_descriptor = msc_fs_configuration_desc,

    #if TUD_OPT_HIGH_SPEED
        .hs_configuration_descriptor = descriptor_hs_cfg_msc,
        .qualifier_descriptor = &descriptor_qualifier_default,

    #endif        
        .string_descriptor = string_desc,
        .string_descriptor_count = sizeof(string_desc)/sizeof(string_desc[0]),
        // .external_phy = false,

        //excluded part of tusb_msc example
        /* #if (TUD_OPT_HIGH_SPEED)
                .fs_configuration_descriptor = msc_fs_configuration_desc,
                .hs_configuration_descriptor = msc_hs_configuration_desc,
                .qualifier_descriptor = &device_qualifier,
            #else 
                .configuration_descriptor = msc_configuration_desc,
            #endif
                */
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    
    /* *tinyusb_msc_spiflash_config_t storage_cfg = {
        .wl_handle = WL_INVALID_HANDLE,   // No wear-levelling by default
        .callback_mount_changed = NULL,
        .callback_premount_changed = NULL,
        .mount_config = {
            .format_if_mount_failed = false,
            .max_files = 4,
            .allocation_unit_size = 4096,
        },
    };

    // Use the storage_cfg variable to avoid unused variable warning
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&storage_cfg));
    */
   
    ESP_LOGI(TAG,"USB MSC initialization DONE");
 
    // UART init
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 
    while(1) tud_task();
}
 