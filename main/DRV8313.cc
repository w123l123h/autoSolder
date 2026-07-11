#include "DRV8313.h"

static const char *TAG = "DRV8313";

DRV8313::DRV8313(gpio_num_t sleep_pin, gpio_num_t reset_pin, gpio_num_t fault_pin, gpio_num_t enable_pin)
    : sleep_pin_(sleep_pin),
      reset_pin_(reset_pin),
      fault_pin_(fault_pin),
      enable_pin_(enable_pin)
{
    ESP_ERROR_CHECK(gpio_set_direction(reset_pin_, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(sleep_pin_, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(enable_pin_, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(fault_pin_, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(fault_pin_, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(reset_pin_, 0));
    ESP_ERROR_CHECK(gpio_set_level(sleep_pin_, 0));
}

void DRV8313::enable(bool enable)
{
    if (enable)
    {
        ESP_ERROR_CHECK(gpio_set_level(reset_pin_, 1));
        ESP_ERROR_CHECK(gpio_set_level(sleep_pin_, 1));
        ESP_ERROR_CHECK(gpio_set_level(enable_pin_, 1));
    }
    else
    {
        ESP_ERROR_CHECK(gpio_set_level(enable_pin_, 0));
    }
}