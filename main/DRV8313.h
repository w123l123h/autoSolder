#ifndef _DRV8313_H_
#define _DRV8313_H_

#include "driver/gpio.h"

class DRV8313
{
public:
    DRV8313(gpio_num_t sleep_pin, gpio_num_t reset_pin, gpio_num_t fault_pin, gpio_num_t enable_pin);

    void enable(bool enable);

private:
    gpio_num_t sleep_pin_ = GPIO_NUM_NC;
    gpio_num_t reset_pin_ = GPIO_NUM_NC;
    gpio_num_t fault_pin_ = GPIO_NUM_NC;
    gpio_num_t enable_pin_ = GPIO_NUM_NC;
};

#endif