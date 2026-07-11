#include "Foc.h"
#include "AngleSensor.h"
#include "Pwm.h"
#include "CurrentSensor.h"
#include "driver/gpio.h"
#include "DRV8313.h"

extern "C" void app_main(void)
{
    DRV8313 drv(GPIO_NUM_47, GPIO_NUM_38, GPIO_NUM_33, GPIO_NUM_35);

    Foc foc;
    foc.init(7, 10, 150, 12);
    foc.q(1);
    foc.d(0);
    foc.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    foc.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    foc.connect(new Pwm(0, 20000, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));

    drv.enable(true);
    foc.start();

    while (true)
    {
        foc.print();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}