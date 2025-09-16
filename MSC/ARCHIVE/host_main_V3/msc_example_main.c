/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

 #include "sdkconfig.h"                 // <-- Added to define config macros
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// USB host headers
#include "usb/usb_host.h"
#include "usb/msc_host_vfs.h"
#include "ff.h"
#include "errno.h"

static const char *TAG = "msc_host_example";

#define MNT_PATH         "/usb"
#define APP_QUIT_PIN     GPIO_NUM_0
#define BUFFER_SIZE      4096
#define MAX_MSC_DEVICES  CONFIG_FATFS_VOLUME_COUNT

typedef struct {
    uint8_t usb_addr;
    msc_host_device_handle_t msc_device;
    msc_host_vfs_handle_t vfs_handle;
} msc_dev_entry_t;

static msc_dev_entry_t *msc_devices[MAX_MSC_DEVICES] = {0};

static QueueHandle_t app_queue;

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

static inline int find_free_slot(void)
{
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devices[i] == NULL) {
            return i;
        }
    }
    return -1;
}

static esp_err_t allocate_new_msc_device(const app_message_t *msg, int *out_slot)
{
    int slot = find_free_slot();
    if (slot < 0) {
        ESP_LOGW(TAG, "No free slots for new MSC device (max %d)", MAX_MSC_DEVICES);
        return ESP_ERR_NOT_FOUND;
    }

    msc_devices[slot] = calloc(1, sizeof(msc_dev_entry_t));
    if (!msc_devices[slot]) {
        ESP_LOGE(TAG, "Failed to allocate memory for new MSC device entry");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = msc_host_install_device(msg->data.new_dev_address, &msc_devices[slot]->msc_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msc_host_install_device failed: %s", esp_err_to_name(err));
        free(msc_devices[slot]);
        msc_devices[slot] = NULL;
        return err;
    }

    msc_devices[slot]->usb_addr = msg->data.new_dev_address;

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 8192,
    };

    char mount_path[16];
    snprintf(mount_path, sizeof(mount_path), MNT_PATH "%d", slot);

    err = msc_host_vfs_register(msc_devices[slot]->msc_device, mount_path, &mount_config, &msc_devices[slot]->vfs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msc_host_vfs_register failed: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK(msc_host_uninstall_device(msc_devices[slot]->msc_device));
        free(msc_devices[slot]);
        msc_devices[slot] = NULL;
        return err;
    }

    *out_slot = slot;
    return ESP_OK;
}

static int find_slot_by_handle(msc_host_device_handle_t handle)
{
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devices[i] && msc_devices[i]->msc_device == handle) {
            return i;
        }
    }
    return -1;
}

static void free_msc_device(int slot)
{
    if (slot < 0 || slot >= MAX_MSC_DEVICES || !msc_devices[slot]) {
        ESP_LOGE(TAG, "Invalid slot index for MSC device deallocation");
        return;
    }

    if (msc_devices[slot]->vfs_handle) {
        ESP_ERROR_CHECK(msc_host_vfs_unregister(msc_devices[slot]->vfs_handle));
    }
    if (msc_devices[slot]->msc_device) {
        ESP_ERROR_CHECK(msc_host_uninstall_device(msc_devices[slot]->msc_device));
    }

    free(msc_devices[slot]);
    msc_devices[slot] = NULL;
}

static void free_all_msc_devices(void)
{
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devices[i]) {
            free_msc_device(i);
        }
    }
}

static void gpio_cb(void *arg)
{
    BaseType_t xTaskWoken = pdFALSE;
    app_message_t message = {.id = APP_QUIT};

    if (app_queue) {
        xQueueSendFromISR(app_queue, &message, &xTaskWoken);
    }

    if (xTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static inline int8_t find_usb_addr_by_handle(msc_host_device_handle_t handle)
{
    for (int8_t i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devices[i] && msc_devices[i]->msc_device == handle) {
            return msc_devices[i]->usb_addr;
        }
    }
    return -1;
}

static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected (usb_addr=%d)", event->device.address);
        app_message_t message = {.id = APP_DEVICE_CONNECTED, .data.new_dev_address = event->device.address};
        xQueueSend(app_queue, &message, portMAX_DELAY);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        int usb_addr = find_usb_addr_by_handle(event->device.handle);
        ESP_LOGI(TAG, "MSC device disconnected (usb_addr=%d)", usb_addr);
        app_message_t message = {.id = APP_DEVICE_DISCONNECTED, .data.device_handle = event->device.handle};
        xQueueSend(app_queue, &message, portMAX_DELAY);
    }
}

static void usb_task(void *args)
{
    const usb_host_config_t host_config = {.intr_flags = ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));

    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &event_flags);
    }

    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

void app_main(void)
{
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    xTaskCreate(usb_task, "usb_task", 4096, NULL, 2, NULL);

    // Init BOOT button
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    ESP_LOGI(TAG, "USB MSC Host example started");

    while (1) {
        app_message_t msg;
        xQueueReceive(app_queue, &msg, portMAX_DELAY);

        if (msg.id == APP_DEVICE_CONNECTED) {
            int slot;
            if (allocate_new_msc_device(&msg, &slot) != ESP_OK) continue;
            ESP_LOGI(TAG, "Device mounted at /usb%d", slot);
        }

        if (msg.id == APP_DEVICE_DISCONNECTED) {
            int slot = find_slot_by_handle(msg.data.device_handle);
            if (slot >= 0) free_msc_device(slot);
        }

        if (msg.id == APP_QUIT) {
            free_all_msc_devices();
            msc_host_uninstall();
            break;
        }
    }

    gpio_isr_handler_remove(APP_QUIT_PIN);
    gpio_uninstall_isr_service();
    vQueueDelete(app_queue);
}
