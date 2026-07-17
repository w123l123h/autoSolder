#include "Application.h"
#include "DRV8313.h"
#include "CurrentSensor.h"
#include "AngleSensor.h"
#include "Pwm.h"
#include "esp_log.h"

#define TAG "Application"

Application::Application()
    : drv_(GPIO_NUM_47, GPIO_NUM_38, GPIO_NUM_33, GPIO_NUM_35)
{
}

void Application::init()
{
    uint32_t freq = 16000;
    uint32_t pidFreq = 1000;
    foc_.init(7, 10, 150, 12, 1, freq / pidFreq);
    foc_.speedPid(0.02, 0.01, 0.0, 10);
    foc_.positionPid(2, 0.000, 0.0, 10);

    drv_.enable(true);
    foc_.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    foc_.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    foc_.connect(new Pwm(0, freq, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));
    foc_.offset(6);
    touch_.init(TOUCH_PAD_NUM1, 10, 10);
    touch_.setCallback([this](TouchEvent event)
                       { ESP_LOGI(TAG, "Touch %s", event == TouchEvent_Touch ? "Touch" : "Release"); });
    //  foc.start();
}

void Application::run()
{
    touch_.run();
}
