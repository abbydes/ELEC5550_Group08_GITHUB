static void init_gpio(void);

//in app_main():

    init_gpio();

//Add this to initialisation section of your main function
#include "driver/gpio.h"

#define LED_PIN     6
static void init_gpio(void)
{
    // Configure LED_PIN as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure INPUT_PIN as input (optional, if you want to read manually)
    io_conf.pin_bit_mask = (1ULL << LASER_RX_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
}

static void led_task(void *arg)
{
    while (1) {
        int level = gpio_get_level(LASER_RX_PIN);

        if (level) {
            gpio_set_level(LED_PIN, 1);  // turn LED ON
        } else {
            gpio_set_level(LED_PIN, 0);  // turn LED OFF
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // check 20 times per second, enough for human eye
    }
}

//Add this to your UART initialization function
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
