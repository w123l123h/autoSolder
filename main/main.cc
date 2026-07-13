#include "Foc.h"
#include "AngleSensor.h"
#include "Pwm.h"
#include "CurrentSensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "DRV8313.h"

const char *TAG = "main";

extern "C" void app_main(void)
{
    DRV8313 drv(GPIO_NUM_47, GPIO_NUM_38, GPIO_NUM_33, GPIO_NUM_35);

    Foc foc;
    foc.init(7, 10, 150, 12, -1);
    foc.speed(18);
    foc.speedPid(0.02, 0.01, 0.0);
    foc.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    foc.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    foc.connect(new Pwm(0, 10000, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));

    drv.enable(true);
    foc.offset(6.25);
    // float offset = foc.updateOffset(3);
    // ESP_LOGI("FOC", "offset: %f", offset);
    // vTaskDelay(pdMS_TO_TICKS(1));
    foc.start();

    float angle = 0;
    while (true)
    {
        foc.print();
        ESP_LOGI(TAG, "speed=%f", foc.totalAngle() - angle);
        angle = foc.totalAngle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}