#include "Foc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "Application.h"

const char *TAG = "main";

extern "C" void app_main(void)
{

    uint32_t freq = 16000;
    uint32_t pidFreq = 1000;

    // Foc foc;
    // foc.init(7, 10, 150, 12, 1, freq / pidFreq);
    // // foc.speed(6.28);
    // foc.speedPid(0.02, 0.01, 0.0, 10);
    // foc.positionPid(2, 0.000, 0.0, 10);
    // foc.position(100);
    // foc.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    // foc.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    // foc.connect(new Pwm(0, freq, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));

    // drv.enable(true);
    // foc.offset(6);
    // // float offset = foc.updateOffset(3);
    // // ESP_LOGI("FOC", "offset: %f", offset);
    // // vTaskDelay(pdMS_TO_TICKS(1));
    // foc.start();

    // float angle = 0;
    // while (true)
    // {
    //     foc.print();
    //     ESP_LOGI(TAG, "speed=%f", foc.totalAngle() - angle);
    //     angle = foc.totalAngle();
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    Application app;
    app.init();
    app.run();
}