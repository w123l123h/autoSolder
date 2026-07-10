#include "Foc.h"
#include "AngleSensor.h"
#include "Pwm.h"
#include "CurrentSensor.h"
#include "driver/gpio.h"

#define DRV_SLEEP_PIN GPIO_NUM_47 // nSLEEP
#define DRV_RESET_PIN GPIO_NUM_38 // nRESET
#define DRV_FAULT_PIN GPIO_NUM_33 // nFAULT (开漏输出，需上拉)
#define DRV_EN GPIO_NUM_35

extern "C" void app_main(void)
{
    //==================== DRV8313RHH 启动 ====================

    gpio_set_direction(DRV_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DRV_SLEEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DRV_RESET_PIN, 0);
    gpio_set_level(DRV_SLEEP_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    gpio_set_level(DRV_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    gpio_set_level(DRV_SLEEP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    gpio_set_direction(DRV_FAULT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DRV_FAULT_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction(DRV_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(DRV_EN, 1);

    Foc foc;
    foc.init(7, 10, 150, 12);
    foc.q(1);
    foc.d(0);
    foc.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    foc.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    foc.connect(new Pwm(0, 20000, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));
    foc.start();
    while (true)
    {
        foc.print();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}