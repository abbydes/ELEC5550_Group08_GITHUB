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
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/msc_host_vfs.h"
#include "errno.h"
#include "driver/gpio.h"
#include "driver/uart.h"

static const char *TAG = "TxMSC";

// ---- Paths / Pins / Limits ----
#define MNT_PATH            "/usb"     // base mount prefix
#define MAX_MSC_DEVICES     CONFIG_FATFS_VOLUME_COUNT

#define LASER_UART_NUM      UART_NUM_1
#define LASER_TX_PIN        15
#define LASER_RX_PIN        16
#define LASER_BAUD          115200     // you can push 921600 if link is solid
#define LASER_BUF_SIZE      1024

// Optional: 5V VBUS enable pin for the USB port power switch (set -1 if not used)
#define VBUS_EN_PIN         -1

// ---- Protocol events ----
static EventGroupHandle_t ack_events;
#define ACK_BIT_CHUNK   (1<<0)
#define ACK_BIT_END     (1<<1)
static volatile int expected_chunk = -1;

// ---- Queues ----
static QueueHandle_t app_queue;      // MSC hotplug messages
typedef struct { char filename[96]; } tx_cmd_t;
static QueueHandle_t tx_cmd_queue;   // “READ <filename>” commands from RX peer

// ---- MSC table ----
typedef struct {
    uint8_t usb_addr;
    msc_host_device_handle_t msc_device;
    msc_host_vfs_handle_t vfs_handle;
} msc_dev_entry_t;

static msc_dev_entry_t *msc_devices[MAX_MSC_DEVICES] = {0};

// ---- App messages ----
typedef struct {
    enum { APP_QUIT, APP_DEVICE_CONNECTED, APP_DEVICE_DISCONNECTED } id;
    union {
        uint8_t new_dev_address;
        msc_host_device_handle_t device_handle;
    } data;
} app_message_t;

// ---------------- Helper functions ----------------
static inline int find_free_slot(void) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) if (!msc_devices[i]) return i;
    return -1;
}

static int find_slot_by_handle(msc_host_device_handle_t handle) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++)
        if (msc_devices[i] && msc_devices[i]->msc_device == handle) return i;
    return -1;
}

static esp_err_t allocate_new_msc_device(const app_message_t *msg, int *out_slot) {
    int slot = find_free_slot();
    if (slot < 0) return ESP_ERR_NOT_FOUND;

    msc_devices[slot] = calloc(1, sizeof(msc_dev_entry_t));
    if (!msc_devices[slot]) return ESP_ERR_NO_MEM;

    esp_err_t err = msc_host_install_device(msg->data.new_dev_address, &msc_devices[slot]->msc_device);
    if (err != ESP_OK) {
        free(msc_devices[slot]); msc_devices[slot] = NULL; return err;
    }
    msc_devices[slot]->usb_addr = msg->data.new_dev_address;

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 4096,
    };

    char mount_path[16];
    snprintf(mount_path, sizeof(mount_path), MNT_PATH "%d", slot);

    err = msc_host_vfs_register(msc_devices[slot]->msc_device, mount_path, &mount_config, &msc_devices[slot]->vfs_handle);
    if (err != ESP_OK) {
        msc_host_uninstall_device(msc_devices[slot]->msc_device);
        free(msc_devices[slot]); msc_devices[slot] = NULL; return err;
    }

    *out_slot = slot;
    ESP_LOGI(TAG, "Mounted MSC device at %s", mount_path);
    return ESP_OK;
}

static void free_msc_device(int slot) {
    if (slot < 0 || slot >= MAX_MSC_DEVICES || !msc_devices[slot]) return;
    if (msc_devices[slot]->vfs_handle) msc_host_vfs_unregister(msc_devices[slot]->vfs_handle);
    if (msc_devices[slot]->msc_device) msc_host_uninstall_device(msc_devices[slot]->msc_device);
    free(msc_devices[slot]); msc_devices[slot] = NULL;
}

// ---------------- USB callbacks ----------------
static void msc_event_cb(const msc_host_event_t *event, void *arg) {
    app_message_t msg;
    switch (event->event) {
        case MSC_DEVICE_CONNECTED:
            msg.id = APP_DEVICE_CONNECTED;
            msg.data.new_dev_address = event->device.address;
            xQueueSend(app_queue, &msg, portMAX_DELAY);
            break;
        case MSC_DEVICE_DISCONNECTED:
            msg.id = APP_DEVICE_DISCONNECTED;
            msg.data.device_handle = event->device.handle;
            xQueueSend(app_queue, &msg, portMAX_DELAY);
            break;
        default: break;
    }
}

// ---------------- UART (Laser link) ----------------
static void init_laser_uart(void) {
    const uart_config_t cfg = {
        .baud_rate = LASER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(LASER_UART_NUM, LASER_BUF_SIZE*2, LASER_BUF_SIZE*2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LASER_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LASER_UART_NUM, LASER_TX_PIN, LASER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART ready @ %d bps", LASER_BAUD);
}

static bool wait_ack_chunk(int chunk_id, TickType_t ticks) {
    expected_chunk = chunk_id;
    EventBits_t b = xEventGroupWaitBits(ack_events, ACK_BIT_CHUNK, pdTRUE, pdTRUE, ticks);
    expected_chunk = -1;
    return (b & ACK_BIT_CHUNK) != 0;
}

static bool wait_ack_end(TickType_t ticks) {
    EventBits_t b = xEventGroupWaitBits(ack_events, ACK_BIT_END, pdTRUE, pdTRUE, ticks);
    return (b & ACK_BIT_END) != 0;
}

// ---------------- File sender ----------------
static void send_file_over_uart(const char *filepath, const char *filename) {
    FILE *fp = fopen(filepath, "rb");                 // binary mode
    if (!fp) { ESP_LOGE(TAG, "File not found: %s", filepath); return; }

    // Announce file
    char header[160];
    snprintf(header, sizeof(header), "FILE:%s\n", filename);
    uart_write_bytes(LASER_UART_NUM, header, strlen(header));

    // stream in chunks
    static uint8_t buf[1024];                         // increase if needed
    int chunk_id = 0;

    while (1) {
        size_t n = fread(buf, 1, sizeof(buf), fp);
        if (n == 0) break;

        char chunkhdr[32];
        snprintf(chunkhdr, sizeof(chunkhdr), "CHUNK %d %u\n", chunk_id, (unsigned)n);
        uart_write_bytes(LASER_UART_NUM, chunkhdr, strlen(chunkhdr));
        uart_write_bytes(LASER_UART_NUM, (const char*)buf, n);

        // wait ACK with retries
        int retries = 5;
        bool ok = false;
        while (retries-- > 0 && !ok) {
            ok = wait_ack_chunk(chunk_id, pdMS_TO_TICKS(2000));
            if (!ok) {
                ESP_LOGW(TAG, "Retry chunk %d", chunk_id);
                uart_write_bytes(LASER_UART_NUM, chunkhdr, strlen(chunkhdr));
                uart_write_bytes(LASER_UART_NUM, (const char*)buf, n);
            }
        }
        if (!ok) { ESP_LOGE(TAG, "Failed chunk %d", chunk_id); fclose(fp); return; }
        chunk_id++;
    }

    fclose(fp);
    uart_write_bytes(LASER_UART_NUM, "END\n", 4);

    if (wait_ack_end(pdMS_TO_TICKS(3000))) {
        ESP_LOGI(TAG, "File %s sent OK", filename);
    } else {
        ESP_LOGW(TAG, "No ACK END");
    }
}

// Task that **only** sends files when commanded
static void file_sender_task(void *arg) {
    tx_cmd_t cmd;
    while (1) {
        if (xQueueReceive(tx_cmd_queue, &cmd, portMAX_DELAY)) {
            // use slot 0 by default
            if (!msc_devices[0]) {
                ESP_LOGW(TAG, "No MSC device mounted");
                continue;
            }
            char filepath[160];
            snprintf(filepath, sizeof(filepath), MNT_PATH "0/%s", cmd.filename);
            send_file_over_uart(filepath, cmd.filename);
        }
    }
}

// UART RX task: parses ASCII lines and sets ACK events; enqueues READ commands
static void uart_rx_task(void *arg) {
    uint8_t rx[LASER_BUF_SIZE];
    char lbuf[256];
    size_t lpos = 0;

    while (1) {
        int len = uart_read_bytes(LASER_UART_NUM, rx, sizeof(rx), pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        for (int i = 0; i < len; ++i) {
            char c = (char)rx[i];
            if (c == '\n' || c == '\r') {
                if (lpos == 0) continue;
                lbuf[lpos] = 0;

                // Commands from RX peer
                if (strncmp(lbuf, "READ ", 5) == 0) {
                    tx_cmd_t cmd = {0};
                    if (sscanf(lbuf + 5, "%95s", cmd.filename) == 1) {
                        xQueueSend(tx_cmd_queue, &cmd, portMAX_DELAY);
                        ESP_LOGI(TAG, "READ request: %s", cmd.filename);
                    }
                }
                // ACKs to our sender
                else if (strncmp(lbuf, "ACK CHUNK ", 10) == 0) {
                    int cid = -1;
                    if (sscanf(lbuf + 10, "%d", &cid) == 1) {
                        if (cid == expected_chunk) {
                            xEventGroupSetBits(ack_events, ACK_BIT_CHUNK);
                        }
                    }
                }
                else if (strcmp(lbuf, "ACK END") == 0) {
                    xEventGroupSetBits(ack_events, ACK_BIT_END);
                }

                lpos = 0; // reset line
            } else {
                if (lpos < sizeof(lbuf) - 1) lbuf[lpos++] = c;
                // else overflow: drop until newline
            }
        }
    }
}

// ---------------- USB Task ----------------
static void usb_task(void *args) {
    // Optional VBUS power
#if VBUS_EN_PIN >= 0
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << VBUS_EN_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(VBUS_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
#endif

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,   // NOTE: correct spelling
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));
    
    // Process core USB host events
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    }
}

// ---------------- App Main ----------------
void app_main(void) {
    app_queue     = xQueueCreate(8, sizeof(app_message_t));
    tx_cmd_queue  = xQueueCreate(4, sizeof(tx_cmd_t));
    ack_events    = xEventGroupCreate();
    assert(app_queue && tx_cmd_queue && ack_events);

    init_laser_uart();
    xTaskCreate(uart_rx_task,     "uart_rx_task",     4096, NULL, 6, NULL);
    xTaskCreate(file_sender_task, "file_sender_task", 4096, NULL, 5, NULL);
    xTaskCreate(usb_task,         "usb_task",         4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Waiting for USB drive + READ commands over UART...");

    while (1) {
        app_message_t msg;
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.id == APP_DEVICE_CONNECTED) {
                int slot;
                if (allocate_new_msc_device(&msg, &slot) == ESP_OK)
                    ESP_LOGI(TAG, "MSC device mounted at /usb%d", slot);
                else
                    ESP_LOGE(TAG, "MSC mount failed");
            } else if (msg.id == APP_DEVICE_DISCONNECTED) {
                int slot = find_slot_by_handle(msg.data.device_handle);
                if (slot >= 0) {
                    ESP_LOGI(TAG, "MSC device removed from /usb%d", slot);
                    free_msc_device(slot);
                }
            } else if (msg.id == APP_QUIT) {
                break;
            }
        }
    }
}
