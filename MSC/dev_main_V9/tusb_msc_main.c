/*
---------------- ELEC5550 - TEAM_08 - msc_dev_V9 ----------------
Module A (Board A from prototyping) software which has the following functionality:
- initialises handshake and reprompts it upon UART bridge interruption
- provides USB-MSC device information to laptop to mimic USB
- writes CBW to the USB flash-drive connected to Module B (Board B)
- receives CSW from Module B (Board B) and passes onto host laptop
*/

//required libraries
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/msc/msc_device.h"
 
static const char *TAG = "boardA_msc_bridge";
 
/*
UART Configuration
- TX pin 17 as per PCB module
- RX pin 18 as per PCB module
- baud rate of 921600 with 70ms latency 
- buf size of 2048 to allow for 512 byte packets to be read
*/
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     18
#define UART_BAUD_RATE  921600
#define UART_BUF_SIZE   2048


//---------------- HANDSHAKE FUNCTIONS ----------------
/*
Handshake for Board A
- sends 'Board A is ready'
- waits until 'Board B is ready' is received
- serial monitor 'Handshake complete with Board B!'
- task deletion after completed
- tracks handshake completion
*/
static TaskHandle_t handshake_task_handle = NULL;
static volatile bool handshake_complete = false;

static void handshake_protocol(void *pvParameters) {
    const char *handshake = "Board A is ready\n";
    const char *expected  = "Board B is ready\n";
 
    uint8_t buf[64];
    TickType_t start_time = xTaskGetTickCount();

    handshake_complete = false;
 
    while (!handshake_complete) {
        // Send Board A handshake message
        uart_write_bytes(UART_PORT_NUM, handshake, strlen(handshake));
        ESP_LOGI(TAG, "Sent handshake: '%s'", handshake);
 
        // Wait for response from Board B
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "Received: '%s'", buf);
 
            // Check for "Board B is ready"
            if (strstr((char *)buf, expected)) {
                handshake_complete = true;
                ESP_LOGI(TAG, "Handshake complete with Board B!");
                uart_flush_input(UART_PORT_NUM);
                vTaskDelete(NULL); // delete task once handshake completed
                
            }
        } else {
            ESP_LOGW(TAG, "No handshake yet, waiting...");
        }
 
        //10s timeout if handshake not established
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(10000)) {
            ESP_LOGW(TAG, "Handshake timeout – retrying...");
            start_time = xTaskGetTickCount();
        }
 
        //minimal task delay to reduce latency
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
Handshake Retrigger for Board A
- triggers upon invalid checksum
- clears UART buffer
- reinitialises handshake protocol
- resets 'handshake_complete'
*/
static void retrigger_handshake(void) {
    ESP_LOGW(TAG, "Retriggering handshake due to invalid checksum");

    handshake_complete = false; //handshake complete set to false as it is retriggered

    uart_flush_input(UART_PORT_NUM); //clear buffer
    xTaskCreate(handshake_protocol, "handshake_protocol", 8196, NULL, 5, &handshake_task_handle); //start new
}


// ---------------- TinyUSB MSC CALLBACKS ----------------
/*
Included from tusb_msc_main.c from the ESP-IDF msc host example 
to emulate required the USB flashdrive information required by 
the laptop for device connection
NOTE: this is where future prototype focus should be spent to ensure
device parameter and drive issues are resolved
*/
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) { return true; }
bool tud_msc_test_unit_ready_cb(uint8_t lun) { return true; }
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    *block_count = 1024;
    *block_size = 512;
}


// ---------------- SERIAL MONITOR ----------------
//used to print tx frames sent to Module B to the serial monitor
static void print_tx_frame(const char *tag, const uint8_t *buf, int len) {
    char line[512];
    char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) {
        p += sprintf(p, "%02X ", buf[i]);
    }
    *p = '\0';
    ESP_LOGI(tag, "UART TX (%d bytes): %s", len, line);
}
 
//used to print rx frames sent from Module B to Module A to the serial monitor
static void print_rx_frame(const char *tag, const uint8_t *buf, int len) {
    if (len <= 0) { ESP_LOGI(tag, "No data received"); return; }
    char line[512];
    char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) {
        p += sprintf(p, "%02X ", buf[i]);
    }
    *p = '\0';
    ESP_LOGI(tag, "UART RX (%d bytes): %s", len, line);
}
 

// ---------------- MSC BLOCK DATA ----------------
/*
This section contains all functions required to both receive and
send the required MSC block data SCSI commands including WRITE10, 
READ10, INQUIRY and a fallback CB for debugging purposes.
*/

/*
Send frame for Board A
- constructs the data being sent from Board A to board B
- adds frame header, type, LBA and payload 
- writes to UART and prints to serial monitor
*/
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {\
    //uart_flush_input(UART_PORT_NUM);

    uint8_t frame[4 + payload_len + 1];
    frame[0] = 0xA5;
    frame[1] = type;
    frame[2] = (payload_len >> 8) & 0xFF;   // high byte
    frame[3] = (payload_len & 0xFF);        // low byte
    memcpy(&frame[4], payload, payload_len);
 
    // checksum = 0x100 - (sum(type + payload_len + payload) & 0xFF)
    uint32_t sum = frame[1] + frame[2] + frame[3];
    for (int i = 0; i < payload_len; i++) sum += frame[4 + i];
    frame[4 + payload_len] = (uint8_t)(0x100 - (sum & 0xFF));
 
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame)); //write to UART
    print_tx_frame(TAG, frame, sizeof(frame)); //print to serial monitor
}
 
/*
CB callback
- explored during prototyping as a potential for sending CBW directly
- did not appear to effect limited function of prototype but useful for debugging
*/
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void* buffer, uint16_t bufsize) {
    // Forward command to Board B
    //uint8_t header[3] = {0xA5, 'C', 16};
    send_frame('C', scsi_cmd, 16);
    ESP_LOGI(TAG, "Forwarded SCSI command to Board B");
    return -1;
}

/*
READ10 callback
- called when READ10 cbw detected
- verifies handshake and sends frame via send_frame
- waits for board B response and prints to serial
- checks frame length, header, type and checksum
- copies sector to laptop via memcpy after LBA removed
- retriggers handshake upon failed checksum
*/
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
 
    //check for handshake completion
    if (!handshake_complete) {
        ESP_LOGW(TAG, "Handshake not complete, cannot process READ10");
        return -1;
    }

    // Payload = 4 bytes LBA
    uint8_t lba_bytes[4] = {
        (uint8_t)(lba >> 24),
        (uint8_t)(lba >> 16),
        (uint8_t)(lba >> 8),
        (uint8_t)(lba)
    };
    send_frame('R', lba_bytes, 4);
    ESP_LOGI(TAG, "Forwarded READ10 to Board B (LBA=%lu, len=%u)", lba, bufsize);
 
    // Wait for Board B to respond with 'D' frame with sector data
    int expected_len = 4 + 4 + bufsize + 1; // header + LBA + sector + checksum
    uint8_t rx_frame[expected_len];  // header + lba + sector + checksum
 
    int len = uart_read_bytes(UART_PORT_NUM, rx_frame, sizeof(rx_frame), pdMS_TO_TICKS(500));
    print_rx_frame(TAG, rx_frame, len);
 
    //frame check
    if (len < 9 || rx_frame[0] != 0xA5 || rx_frame[1] != 'D') {
        ESP_LOGW(TAG, "READ10 invalid frame (len = %d)", len);
        retrigger_handshake();
        return -1;
    }

    //checksum validation
    uint16_t payload_len = (rx_frame[2] << 8) | rx_frame[3];
    uint32_t sum = rx_frame[1] + rx_frame[2] + rx_frame[3];

    for (int i = 0; i < payload_len; i++) sum += rx_frame[4 + i];
    uint8_t checksum = (uint8_t)(0x100 - (sum & 0xFF));

    if (rx_frame[len - 1] != checksum) {
        ESP_LOGW(TAG, "Checksum mismatch in READ10");
        retrigger_handshake();
        return -1;
    }

    //copying one sector to laptop
    uint16_t data_len = payload_len - 4; // remove LBA
    if (data_len > bufsize) data_len = bufsize;
    memcpy(buffer, &rx_frame[8], data_len);

    ESP_LOGI(TAG, "READ10 successful (LBA=%lu, bytes=%u)", lba, data_len);
    vTaskDelay(pdMS_TO_TICKS(1)); //minimal delay to reduce latency
    return data_len;
}
 
 
/*
WRITE10 callback
- called when WRITE10 cbw detected
- verifies handshake and sends frame via send_frame
- waits for board B ACK response and prints to serial
- checks frame length, header, type and checksum
- retriggers handshake upon incorrect checksum
*/
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    
    //check for handshake completion
    if (!handshake_complete) {
        ESP_LOGW(TAG, "Handshake not complete, cannot process WRITE10");
        return -1;
    }
    // Payload = 4 bytes LBA + sector data
    uint8_t payload[4 + bufsize];
    payload[0] = (uint8_t)(lba >> 24);
    payload[1] = (uint8_t)(lba >> 16);
    payload[2] = (uint8_t)(lba >> 8);
    payload[3] = (uint8_t)(lba);
    memcpy(&payload[4], buffer, bufsize);
 
    send_frame('W', payload, 4 + bufsize);
    ESP_LOGI(TAG, "Forwarded WRITE10 to Board B (LBA=%lu, len=%u)", lba, bufsize);
 
    // Wait for ACK (0xAA)
    uint8_t ack;
    int len = uart_read_bytes(UART_PORT_NUM, &ack, 1, pdMS_TO_TICKS(500));
    if (len == 1 && ack == 0xAA) {
        ESP_LOGI(TAG, "WRITE10 successful (LBA = %lu)", lba);
        return bufsize;
    }

    ESP_LOGW(TAG, "WRITE10 failed: no ACK");
    retrigger_handshake();
    return -1;
}
 
/*
INQUIRY callback
- provides laptop with required vid, pid and rev which are necessary
  for USB initialisation
NOTE: similar to the MSC callbacks, may require updating for full
USB device functionality
*/
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    const char vid[] = "BoardA  ";
    const char pid[] = "MSC Bridge     ";
    const char rev[] = "1.0 ";
    memcpy(vendor_id, vid, sizeof(vid)-1);
    memcpy(product_id, pid, sizeof(pid)-1);
    memcpy(product_rev, rev, sizeof(rev)-1);
}

// ---------------- APP MAIN ----------------
void app_main(void) {
    ESP_LOGI(TAG, "Board A MSC Transparent Bridge starting...");
 
    // UART initalisation with parameters outlined above
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //install driver as per outlined configuration and set TX & RX pins
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 
    //Run initialising handshake 
    xTaskCreate(handshake_protocol, "handshake_protocol", 8196, NULL, 5, &handshake_task_handle);
 
    //Initiate tinyusb configuration
    //NOTE: May require updating for full device functionality
    tinyusb_config_t tusb_cfg = {0};
    //tusb_cfg.device_descriptor = NULL; //for descriptors
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB initialized");
 
    while (1) {
        tud_task(); //handles USB events by calling READ10, WRITE10, INQUIRY, etc.
    }
}
