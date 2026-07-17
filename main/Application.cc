#include "Application.h"
#include "DRV8313.h"
#include "CurrentSensor.h"
#include "AngleSensor.h"
#include "Pwm.h"
#include "esp_log.h"
#include "freertos/timers.h" // 必须包含定时器头文件
#include <cmath>

#define TAG "Application"

Application::Application()
    : drv_(GPIO_NUM_47, GPIO_NUM_38, GPIO_NUM_33, GPIO_NUM_35)
{
}

void Application::init()
{
    uint32_t freq = 16000;
    uint32_t pidFreq = 1000;

    foc_.connect(new AngleSensor(GPIO_NUM_10, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_11));
    foc_.connect(new CurrentSensor(ADC_CHANNEL_3, ADC_CHANNEL_5));
    foc_.connect(new Pwm(0, freq, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_37));

    foc_.speedPid(0.02, 0.01, 0.0, 8, 10);
    foc_.positionPid(2, 0.000, 0.0, 10, 18);

    foc_.init(7, 10, 150, 12, 1, freq / pidFreq);
    foc_.offset(6);

    touch_.init(TOUCH_PAD_NUM1, 10, 10);
    touch_.setCallback(
        [this](TouchEvent event)
        {
            ESP_LOGI(TAG, "Touch %s", event == TouchEvent_Touch ? "Touch" : "Release");
            if (event == TouchEvent_Touch)
            {
                touched();
            }
            else
            {
                released();
            }
        });

    timer_ = xTimerCreate(
        "Waiting",           // 定时器名称（调试用）
        pdMS_TO_TICKS(3000), // 定时周期：3000ms 转换为 tick 数
        pdFALSE,             // 是否自动重载：pdFALSE 表示单次触发
        (void *)this,        // 传递给回调的用户参数（可选）
        timerCallback        // 回调函数
    );
}

void Application::run()
{
    touch_.run();
    foc_.start();
    float angle = 0;
    while (true)
    {
        foc_.print();
        ESP_LOGI(TAG, "speed=%f", foc_.totalAngle() - angle);
        angle = foc_.totalAngle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Application::touched()
{
    drv_.enable(false);
    switch (state_)
    {
    case WorkState::Idle:
    {
        break;
    }
    case WorkState::Waiting:
        xTimerStop(timer_, 0);
        break;
    case WorkState::Using:
        assert(false);
        break;
    case WorkState::Recycling:
        // 重新计算changed_angle_
        vTaskDelete(task_);
        changed_angle_ = foc_.position() - target_position_;
        break;
    }

    state_ = WorkState::Using;
    position_ = foc_.position();
    ESP_LOGI(TAG, "Touched= %f", position_);
}

void Application::released()
{
    changed_angle_ += foc_.position() - position_ + offset_;
    state_ = WorkState::Waiting;

    xTimerStart(timer_, 0);
    ESP_LOGI(TAG, "Released= %f, changed angle= %f", foc_.position(), changed_angle_);
}

void Application::recycled()
{
    assert(state_ == WorkState::Waiting);
    state_ = WorkState::Recycling;
    float cur = foc_.position();
    target_position_ = cur - changed_angle_;

    foc_.resetPosition();
    foc_.resetSpeed();
    foc_.position(target_position_);
    drv_.enable(true);
    ESP_LOGI(TAG, "Recycled= %f", target_position_);

    xTaskCreatePinnedToCore(task, "CheckRecycled", 4096, this, 5, &task_, 0);
}

void Application::checkRecycled()
{
    while (true)
    {
        assert(state_ == WorkState::Recycling);
        if (fabs(foc_.position() - target_position_) < 0.005)
        {
            state_ = WorkState::Idle;
            drv_.enable(false);
            changed_angle_ = 0;
            ESP_LOGI(TAG, "Finished= %f", target_position_);
            vTaskDelete(task_);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Application::timerCallback(TimerHandle_t timer)
{
    Application *app = (Application *)pvTimerGetTimerID(timer);
    assert(app);
    app->recycled();
}

void Application::task(void *p)
{
    Application *app = (Application *)p;
    assert(app);
    app->checkRecycled();
}
