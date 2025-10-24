#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/msc_host_vfs.h"
#include "ffconf.h"
#include "errno.h"
#include "driver/gpio.h"
#include "driver/uart.h"

static const char *TAG = "TxMSC";

#define MNT_PATH         "/usb"     // Base mount path prefix
#define APP_QUIT_PIN     GPIO_NUM_0
#define MAX_MSC_DEVICES  CONFIG_FATFS_VOLUME_COUNT

// ---------------- UART (Laser link) ----------------
#define LASER_UART_NUM   UART_NUM_1
#define LASER_TX_PIN     15
#define LASER_RX_PIN     16
#define LASER_BAUD       115200
#define LASER_BUF_SIZE   1024

// MSC device table
typedef struct {
    uint8_t usb_addr;
    msc_host_device_handle_t msc_device;
    msc_host_vfs_handle_t vfs_handle;
} msc_dev_entry_t;

static msc_dev_entry_t *msc_devices[MAX_MSC_DEVICES] = {0};
static QueueHandle_t app_queue;

// App message type
typedef struct {
    enum {
        APP_QUIT,
        APP_DEVICE_CONNECTED,
        APP_DEVICE_DISCONNECTED,
    } id;
    union {
        uint8_t new_dev_address;
        msc_host_device_handle_t device_handle;
    } data;
} app_message_t;

// ---------------- Helper functions ----------------
static inline int find_free_slot(void) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (!msc_devices[i]) return i;
    }
    return -1;
}

static int find_slot_by_handle(msc_host_device_handle_t handle) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devices[i] && msc_devices[i]->msc_device == handle) return i;
    }
    return -1;
}

static esp_err_t allocate_new_msc_device(const app_message_t *msg, int *out_slot) {
    int slot = find_free_slot();
    if (slot < 0) return ESP_ERR_NOT_FOUND;

    msc_devices[slot] = calloc(1, sizeof(msc_dev_entry_t));
    if (!msc_devices[slot]) return ESP_ERR_NO_MEM;

    esp_err_t err = msc_host_install_device(msg->data.new_dev_address, &msc_devices[slot]->msc_device);
    if (err != ESP_OK) {
        free(msc_devices[slot]);
        msc_devices[slot] = NULL;
        return err;
    }
    msc_devices[slot]->usb_addr = msg->data.new_dev_address;

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 4096,
    };

    char mount_path[16];
    snprintf(mount_path, sizeof(mount_path), MNT_PATH "%d", slot);

    err = msc_host_vfs_register(msc_devices[slot]->msc_device, mount_path, &mount_config, &msc_devices[slot]->vfs_handle);
    if (err != ESP_OK) {
        msc_host_uninstall_device(msc_devices[slot]->msc_device);
        free(msc_devices[slot]);
        msc_devices[slot] = NULL;
        return err;
    }

    *out_slot = slot;
    ESP_LOGI(TAG, "Mounted MSC device at %s", mount_path);
    return ESP_OK;
}

static void free_msc_device(int slot) {
    if (slot < 0 || slot >= MAX_MSC_DEVICES || !msc_devices[slot]) return;
    if (msc_devices[slot]->vfs_handle) {
        msc_host_vfs_unregister(msc_devices[slot]->vfs_handle);
    }
    if (msc_devices[slot]->msc_device) {
        msc_host_uninstall_device(msc_devices[slot]->msc_device);
    }
    free(msc_devices[slot]);
    msc_devices[slot] = NULL;
}

// ---------------- USB event callback ----------------
static void msc_event_cb(const msc_host_event_t *event, void *arg) {
    if (event->event == MSC_DEVICE_CONNECTED) {
        app_message_t msg = {.id = APP_DEVICE_CONNECTED, .data.new_dev_address = event->device.address};
        xQueueSend(app_queue, &msg, portMAX_DELAY);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        app_message_t msg = {.id = APP_DEVICE_DISCONNECTED, .data.device_handle = event->device.handle};
        xQueueSend(app_queue, &msg, portMAX_DELAY);
    }
}

// ---------------- Laser UART ----------------
static void init_laser_uart(void) {
    const uart_config_t cfg = {
        .baud_rate = LASER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(LASER_UART_NUM, LASER_BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LASER_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LASER_UART_NUM, LASER_TX_PIN, LASER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Laser UART ready");
}

// Send file over UART
// --- inside Tx code ---
static void send_file_over_uart(const char *filepath, const char *filename) {
    FILE *fp = fopen(filepath, "r");
    if (!fp) {
        ESP_LOGE(TAG, "File %s not found", filepath);
        return;
    }

    // Announce file
    char header[128];
    snprintf(header, sizeof(header), "FILE:%s\n", filename);
    uart_write_bytes(LASER_UART_NUM, header, strlen(header));

    char buf[256];
    int chunk_id = 0;
    while (1) {
        size_t n = fread(buf, 1, sizeof(buf), fp);
        if (n == 0) break;

        // Send CHUNK header
        char chunkhdr[32];
        snprintf(chunkhdr, sizeof(chunkhdr), "CHUNK %d %u\n", chunk_id, (unsigned)n);
        uart_write_bytes(LASER_UART_NUM, chunkhdr, strlen(chunkhdr));
        uart_write_bytes(LASER_UART_NUM, buf, n);

        // Wait for ACK
        char ackbuf[64];
        int retries = 3;
        bool acked = false;
        while (retries-- > 0 && !acked) {
            int len = uart_read_bytes(LASER_UART_NUM, (uint8_t*)ackbuf, sizeof(ackbuf)-1,
                                      500 / portTICK_PERIOD_MS);
            if (len > 0) {
                ackbuf[len] = 0;
                char expected[32];
                snprintf(expected, sizeof(expected), "ACK CHUNK %d", chunk_id);
                if (strstr(ackbuf, expected)) {
                    acked = true;
                    ESP_LOGI(TAG, "Chunk %d acked", chunk_id);
                }
            }
            if (!acked && retries > 0) {
                ESP_LOGW(TAG, "Retry chunk %d", chunk_id);
                uart_write_bytes(LASER_UART_NUM, chunkhdr, strlen(chunkhdr));
                uart_write_bytes(LASER_UART_NUM, buf, n);
            }
        }
        if (!acked) {
            ESP_LOGE(TAG, "Failed to send chunk %d", chunk_id);
            fclose(fp);
            return;
        }
        chunk_id++;
    }

    fclose(fp);
    uart_write_bytes(LASER_UART_NUM, "END\n", 4);

    // Final ACK
    char ackbuf[32];
    int len = uart_read_bytes(LASER_UART_NUM, (uint8_t*)ackbuf, sizeof(ackbuf)-1,
                              1000 / portTICK_PERIOD_MS);
    if (len > 0 && strstr(ackbuf, "ACK END")) {
        ESP_LOGI(TAG, "File %s sent successfully", filename);
    } else {
        ESP_LOGW(TAG, "No ACK END received");
    }
}

// UART task to listen for "READ filename"
static void laser_uart_task(void *arg) {
    uint8_t data[LASER_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(LASER_UART_NUM, data, LASER_BUF_SIZE-1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0;
            ESP_LOGI(TAG, "UART RX: %s", (char*)data);

            if (strncmp((char*)data, "READ ", 5) == 0) {
                char filename[64];
                sscanf((char*)data+5, "%63s", filename);

                // Always use slot 0 (first MSC device)
                if (msc_devices[0]) {
                    char filepath[128];
                    snprintf(filepath, sizeof(filepath), MNT_PATH "0/%s", filename);
                    send_file_over_uart(filepath, filename);
                } else {
                    ESP_LOGW(TAG, "No MSC device mounted, cannot read");
                }
            }
        }
    }
}

// ---------------- USB Task ----------------
static void usb_task(void *args) {
    const usb_host_config_t host_config = {.intr_flags = ESP_INTR_FLAG_LEVEL1};
    usb_host_install(&host_config);

    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    msc_host_install(&msc_config);

    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    }
}

// ---------------- App Main ----------------
void app_main(void) {
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    init_laser_uart();
    xTaskCreate(laser_uart_task, "laser_uart_task", 4096, NULL, 5, NULL);
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Waiting for USB drive + UART requests...");

    while (1) {
        app_message_t msg;
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.id == APP_DEVICE_CONNECTED) {
                int slot;
                if (allocate_new_msc_device(&msg, &slot) == ESP_OK) {
                    ESP_LOGI(TAG, "MSC device mounted at /usb%d", slot);
                }
            } else if (msg.id == APP_DEVICE_DISCONNECTED) {
                int slot = find_slot_by_handle(msg.data.device_handle);
                if (slot >= 0) {
                    free_msc_device(slot);
                }
            } else if (msg.id == APP_QUIT) {
                break;
            }
        }
    }
}
