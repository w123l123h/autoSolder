#ifndef _ANGLE_SENSOR_H_
#define _ANGLE_SENSOR_H_

#include "IAngleSensor.h"
#include "soc/gpio_num.h"
#include "driver/spi_master.h"

class AngleSensor : public IAngleSensor
{
public:
    AngleSensor(gpio_num_t cs_pin, gpio_num_t sck_pin, gpio_num_t miso_pin, gpio_num_t mosi_pin);

    float getAngle();

private:
    int cs_pin_ = 0;
    int sck_pin_ = 0;
    int miso_pin_ = 0;
    int mosi_pin_ = 0;
    spi_device_handle_t spi_dev_ = nullptr;
};

#endif