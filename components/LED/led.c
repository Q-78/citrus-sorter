#include "led.h"
#include "driver/gpio.h"

void led_init(void)
{
    gpio_config_t gpioConfig = {
        .pin_bit_mask = 1ULL << GPIO_NUM_38, // Configure GPIO38
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&gpioConfig);
    if (err != ESP_OK) {
        printf("GPIO configuration failed with error code: %d\n", err);
    }
}