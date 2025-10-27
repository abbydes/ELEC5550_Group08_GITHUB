/*
---------------- ELEC5550 - TEAM_08 - host_main_V8/msc_example_main ----------------
Module B (Board B from prototyping) software which has the following functionality:
- initialises handshake and reprompts it upon UART bridge interruption
- mounts USB flash-drive to allow for MSC block/sector data transfer
- receives CBW from Module A and passes on to USB flash-drive
- sends CSW extracted from USB flash-drive with error correction
*/

//required libraries
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "esp_private/msc_scsi_bot.h"

static const char *TAG = "msc_host_boardB";

/*
UART Configuration
- TX pin 17 as per PCB module
- RX pin 18 as per PCB module
- baud rate of 921600 with 70ms latency 
- buf size of 2048 to allow for 512 byte packets to be read
*/
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     921600
#define UART_TX_PIN        17
#define UART_RX_PIN        18
#define UART_BUF_SIZE      2048

/*
RX Frame Configuration
- outlines size of rx and tx frames
- defines sector size as 512 bytes
- initialises UART queue for uart_rx_task
*/
#define RX_FRAME_SIZE (3 + 4 + 512 + 1)
static uint8_t rx_frame[3 + 4 + 512 + 1];
static uint8_t tx_frame[3 + 4 + 512 + 1];
static uint8_t sector[512];
static QueueHandle_t uart_queue;


//---------------- HANDSHAKE FUNCTIONS ----------------
/*
Handshake Configuration
- sets a task handle, complete and trigger bools as well as a handshake countdown
*/
static TaskHandle_t handshake_task_handle = NULL;
static volatile bool handshake_complete = false;
static volatile bool handshake_triggered = false;
static uint32_t last_handshake_ms = 0;

/*
Handshake for Board B
- waits until 'Board A is ready' is received
- replies with 'Board B is ready'
- serial monitor 'Handshake complete with Board B!'
- task deletion after completed
- tracks handshake completion
- has 'retrigger' functionality
*/
static void handshake_protocol(void *pvParameters) {
    const char *expected = "Board A is ready\n";
    const char *reply = "Board B is ready\n";
    uint8_t buf[64];
 
    ESP_LOGI(TAG, "Waiting for handshake from Board A...");
    handshake_complete = false;

    while (!handshake_complete) {
        // handshake retrigger
        if (handshake_triggered) {
            ESP_LOGW(TAG, "Retriggering handshake protocol as 'Board A is ready' received.");
            // send Board B handshake message
            uart_write_bytes(UART_PORT_NUM, reply, strlen(reply));
            ESP_LOGI(TAG, "Replied handshake: '%s'", reply);
            ESP_LOGI(TAG, "Handshake complete with Board A!");

            uart_flush_input(UART_PORT_NUM);
            handshake_triggered = false; // handshake retrigger finalised
            handshake_complete = true; // handshake completed
            handshake_task_handle = NULL;
            vTaskDelete(NULL); // delete task once handshake completed
        }

        // Wait to receive the 'Board A is ready' message (start)
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "Received: '%s'", buf);
 
            if (strstr((char *)buf, expected)) {
                //Respond once Board A announces readiness
                uart_write_bytes(UART_PORT_NUM, reply, strlen(reply));
                ESP_LOGI(TAG, "Replied handshake: '%s'", reply);
                ESP_LOGI(TAG, "Handshake complete with Board A!");
                uart_flush_input(UART_PORT_NUM);

                handshake_complete = true;
                handshake_task_handle = NULL;
                vTaskDelete(NULL); // delete task once handshake completed
            }
 
        } else {
            ESP_LOGW(TAG, "No handshake yet, waiting...");
        }
 

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
Handshake Retrigger for Board B
- triggers upon 'Board A is ready' being received
- reinitialises handshake protocol
- resets 'handshake_complete'
*/
//handshake retrigger for Board B receiving Board A handshake
static void retrigger_handshake(void) {
    ESP_LOGW(TAG, "Retriggering handshake protocol as 'Board A is ready' received.");
    handshake_complete = false;

    if (handshake_task_handle == NULL) {
        handshake_triggered = true;
        xTaskCreate(handshake_protocol, "handshake_protocol", 8196, NULL, 6, &handshake_task_handle); //restart handshake
    } else {
        // Signal the running handshake task to restart
        handshake_triggered = true;
    }
}

//---------------- MSC EVENTS ----------------
/*
Included from msc_example_main.c from the ESP-IDF msc host example 
to emulate required the USB flashdrive mounting to the module
*/

// App queue and message types for MSC events
typedef enum { APP_DEVICE_CONNECTED, APP_DEVICE_DISCONNECTED } app_msg_id_t;
typedef struct {
    app_msg_id_t id;
    union {
        uint8_t new_dev_address;             // on connect
        msc_host_device_handle_t device;     // on disconnect
    } data;
} app_message_t;
static QueueHandle_t app_queue;

// Single-device handle
static msc_host_device_handle_t msc_dev = NULL;

// MSC callback: posts events to app queue
static void msc_event_cb(const msc_host_event_t *event, void *arg) {
    app_message_t msg;
    if (!app_queue) return;
    if (event->event == MSC_DEVICE_CONNECTED) {
        msg.id = APP_DEVICE_CONNECTED;
        msg.data.new_dev_address = event->device.address;
        xQueueSendFromISR(app_queue, &msg, NULL);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        msg.id = APP_DEVICE_DISCONNECTED;
        msg.data.device = event->device.handle;
        xQueueSendFromISR(app_queue, &msg, NULL);
    }
}

// USB host task
static void usb_task(void *arg) {
    ESP_LOGI(TAG, "usb_task started");

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .skip_phy_setup = false,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,   
        .task_priority = 5,
        .stack_size = 8192, // increased stack to account for sectors
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_cfg));
    ESP_LOGI(TAG, "USB Host and MSC driver installed");

    while (1) {
        esp_err_t err = usb_host_lib_handle_events(50, NULL);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
            ESP_LOGE(TAG, "usb_host_lib_handle_events() err=%s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ---------------- SERIAL MONITOR ----------------
//used to print tx frames sent to Module A to the serial monitor
static void print_rx_frame(const char *tag, const uint8_t *buf, int len) {
    if (len <= 0) { ESP_LOGI(tag, "No packets received from Board A"); return; }
    char line[1024]; char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) p += sprintf(p, "%02X ", buf[i]);
    *p = '\0'; ESP_LOGI(tag, "UART RX (%d bytes): %s", len, line);
}

//used to print rx frames sent from Module A to Module A tB the serial monitor
static void print_tx_frame(const char *tag, const uint8_t *buf, int len) {
    char line[512]; char *p = line;
    for (int i = 0; i < len && p < line + sizeof(line) - 4; i++) p += sprintf(p, "%02X ", buf[i]);
    *p = '\0'; ESP_LOGI(tag, "UART TX (%d bytes): %s", len, line);
}

// ---------------- UART RX TASK ----------------
/*
Task used to read incoming sectors from Module A and extract required
response sectors from the USB flash-drive mounted on Module B. Includes:
- check for handshake retrigger
- frame reading including header, LBA, checksum and payload
- READ10 and WRITE10 handling
*/
static void uart_rx_task(void *arg) {
    while (!msc_dev) {
        ESP_LOGI(TAG, "Waiting for MSC device...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "MSC device ready, starting UART processing");


    uint8_t byte;
    static char text_buf[64];
    int text_pos = 0;


    while (1) {
        // Read and log every incoming byte for handshake retrigger
        int len = uart_read_bytes(UART_PORT_NUM, &byte, 1, pdMS_TO_TICKS(1));
        if (len > 0) {
            // Collect ASCII text for handshake retrigger
            if (byte >= 0x20 && byte <= 0x7E) {
                if (text_pos < sizeof(text_buf) - 1) text_buf[text_pos++] = byte;
            } else if (byte == '\n' || byte == '\r') {
                if (text_pos > 0) {
                    text_buf[text_pos] = '\0';
                    if (strstr(text_buf, "Board A is ready")) {
                        ESP_LOGW(TAG, "Detected handshake trigger!");
                        retrigger_handshake();
                    }
                }
                text_pos = 0;
            }

            if (!handshake_complete) {
                vTaskDelay(pdMS_TO_TICKS(1)); //minimise latency
                continue; 
            }
 
        }

        // frame reading
        // Wait for start byte
        uint8_t start = 0;
        while (start != 0xA5) {
            if (uart_read_bytes(UART_PORT_NUM, &start, 1, pdMS_TO_TICKS(100)) <= 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
            } 

            // Detect if handshake message starts mid-frame
            if (start >= 0x20 && start <= 0x7E) {
                // Collect potential handshake string
                text_buf[0] = start;
                int ascii_len = uart_read_bytes(UART_PORT_NUM, &text_buf[1], sizeof(text_buf) - 2, pdMS_TO_TICKS(20));
                if (ascii_len > 0) {
                    text_buf[ascii_len + 1] = '\0';
                    if (strstr(text_buf, "Board A is ready")) {
                        //timer for handshake string read
                        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

                        if (now - last_handshake_ms > 100) {
                            last_handshake_ms = now;
                            ESP_LOGW(TAG, "Detected handshake during frame read!");
                            uart_flush_input(UART_PORT_NUM);

                            //reset data
                            memset(rx_frame, 0, sizeof(rx_frame));
                            handshake_triggered = false;
                            handshake_complete = false;
                            retrigger_handshake();
                            continue;
                        }
                    }
                }
            }
        }

        if (handshake_triggered && !handshake_complete) {
            retrigger_handshake();
            handshake_triggered = false;
            continue; // restart frame read after handshake
        }

        // reading rest of the frame
        // header
        uint8_t header[3];
        if (uart_read_bytes(UART_PORT_NUM, header, sizeof(header), pdMS_TO_TICKS(100)) != sizeof(header)) {
            ESP_LOGW(TAG, "Failed to read header");
            continue;
        }
        uint8_t type = header[0];
        uint16_t payload_len = (header[1]<<8) | header[2];

        // payload + checksum
        uint8_t frame[payload_len + 1];
        int total_read = 0, timeout = 0;
        while (total_read < payload_len + 1 && timeout < 100) {
            int r = uart_read_bytes(UART_PORT_NUM, &frame[total_read], payload_len + 1 - total_read, pdMS_TO_TICKS(1));
            if (r > 0) {
                print_rx_frame(TAG, &frame[total_read], r); // print incoming payload chunk
                total_read += r;
            } else {
                timeout++;
            }
        }
        
        //checksum fail
        if (total_read < payload_len + 1) {
            ESP_LOGW(TAG, "Incomplete frame received (got %d/%d)", total_read, payload_len + 1);
            continue;
        }

        //READ10 and WRITE10 handling
        //NOTE: may have to be expanded to account for other SCSI commands for
        //full USB flash-drive functionality
        if (type == 'R') { //READ10
            uint32_t lba = (rx_frame[4]<<24)|(rx_frame[5]<<16)|(rx_frame[6]<<8)|rx_frame[7];

            //extracting sector from USB flash-drive
            esp_err_t ret = scsi_cmd_read10(msc_dev, sector, lba, 1, sizeof(sector));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SCSI READ10 failed");
                continue;
            }

            //constructing frame to send back
            uint16_t resp_payload_len = 4 + sizeof(sector);
            int total_len = 4 + resp_payload_len + 1;

            tx_frame[0] = 0xA5; //header
            tx_frame[1] = 'D'; //type
            tx_frame[2] = (resp_payload_len >> 8) & 0xFF; // high byte
            tx_frame[3] = (resp_payload_len & 0xFF); //low byte
            memcpy(&tx_frame[4], &lba, 4);
            memcpy(&tx_frame[8], sector, sizeof(sector));

            // checksum = 0x100 - (sum(type + payload_len + payload) & 0xFF)
            uint32_t sum = tx_frame[1] + tx_frame[2] + tx_frame[3];
            for(int i = 0; i < resp_payload_len; i++) sum += tx_frame[4 + i];
            tx_frame[4 + resp_payload_len]=(uint8_t)(0x100 - (sum & 0xFF));

            uart_write_bytes(UART_PORT_NUM, (const char*)tx_frame, total_len);
            print_tx_frame(TAG, tx_frame, total_len);

        } else if (type == 'W') { //WRITE10
            uint32_t lba = (rx_frame[4]<<24)|(rx_frame[5]<<16)|(rx_frame[6]<<8)|rx_frame[7];
            memcpy(sector, &rx_frame[8], sizeof(sector));

            //extracting sector from USB flash-drive
            esp_err_t ret = scsi_cmd_write10(msc_dev, sector, lba, 1, sizeof(sector));
            if( ret != ESP_OK) {
                ESP_LOGE(TAG, "SCSI WRITE10 failed");
                continue;
            }

            //writing ack back to Module A
            uint8_t ack = 0xAA;
            uart_write_bytes(UART_PORT_NUM,&ack,1);
            print_tx_frame(TAG,&ack,1);
        }
    }
}


// ---------------- APP MAIN ----------------
void app_main(void) {
    ESP_LOGI(TAG,"Board B USB MSC Host (bridge) starting...");

    // UART initalisation with parameters outlined above
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //install driver as per outlined configuration and set TX & RX pins
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM,&uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //Run initialising handshake 
    xTaskCreate(handshake_protocol, "handshake_protocol", 8196, NULL, 6, &handshake_task_handle);

    //Create USB flash-drive Queue
    app_queue = xQueueCreate(5,sizeof(app_message_t));
    if(!app_queue) return;

    //Initialise USB and UART tasks
    xTaskCreate(usb_task,"usb_task",8192,NULL,5,NULL);
    xTaskCreate(uart_rx_task,"uart_rx_task",8192,NULL,5,NULL);

    //run USB flash-drive mounting loop
    while(1){
        app_message_t msg;
        if(xQueueReceive(app_queue,&msg,portMAX_DELAY)==pdTRUE){
            if(msg.id==APP_DEVICE_CONNECTED){
                ESP_LOGI(TAG,"APP: Device connected -> installing (addr=%d)", msg.data.new_dev_address);
                if(msc_host_install_device(msg.data.new_dev_address,&msc_dev)!=ESP_OK){
                    msc_dev=NULL;
                }
            } else if(msg.id==APP_DEVICE_DISCONNECTED){
                if(msc_dev){
                    msc_host_uninstall_device(msc_dev);
                    msc_dev=NULL;
                }
            }
        }
    }
}

