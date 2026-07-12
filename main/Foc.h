#ifndef _FOC_H
#define _FOC_H

#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "IAngleSensor.h"
#include "ICurrentSensor.h"
#include "IPwm.h"

class Pid
{
public:
    void t(float t)
    {
        t_ = t;
    }

    void pid(float kp, float ki, float kd)
    {
        enable_ = true;
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    float update(float c)
    {
        err_ = t_ - c;
        err_total_ += err_;
        float rt = kp_ * err_ + ki_ * err_total_ + kd_ * (err_ - err_last_);
        err_last_ = err_;
        return rt;
    }

private:
    bool enable_ = false;
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float err_ = 0.0f;
    float err_last_ = 0.0f;
    float err_total_ = 0.0f;
    float t_ = 0.0f;
};

class Foc
{
public:
    void init(int pair, float r, int kv, float dc);

    void print() const;

    float updateOffset(float d);
    void offset(float offset)
    {
        offset_ = offset;
    }
    void speedPid(float p, float i, float d)
    {
        pid_speed_.pid(p, i, d);
    }
    void positionPid(float p, float i, float d)
    {
        pid_position_.pid(p, i, d);
    }
    void speed(float speed)
    {
        pid_speed_.t(speed);
    }

    float totalAngle() const
    {
        return total_angle_;
    }

    void start();
    void stop();

    void connect(IAngleSensor *);
    void connect(ICurrentSensor *);
    void connect(IPwm *);

private:
    static void foc_task(void *);
    static void adc_task(void *);
    static void pwm_func(void *p);
    static void adc_func(void *p);
    void update();
    void update_duty();

private:
    int pair_ = 0;
    float r_ = 0.0f;
    int kv_ = 0;
    float dc_ = 0.0f;
    uint32_t freq_hz_ = 0;
    float offset_ = 0.0f;
    volatile bool update_offset_ = false;
    volatile float offset_step_ = 0.0003f;

    volatile float tq_ = 0.0f;
    volatile float td_ = 0.0f;

    float q_ = 0.0f;
    float d_ = 0.0f;

    int pid_count_ = 0;
    float pid_angle_ = 0.0f;
    int64_t pid_ts_ = 0;
    int pid_ts_internal_ = 0;
    Pid pid_speed_;
    Pid pid_position_;

    adc_oneshot_unit_handle_t adc_handle = nullptr;
    adc_cali_handle_t cali_handle = nullptr;

    volatile float current_ma_a_ = 0;
    volatile float current_ma_b_ = 0;
    volatile float current_ma_c_ = 0;
    float angle_ = 0.0;     // 电角度
    float angle_rad_ = 0.0; // 转子角度
    float last_angle_ = -1.0;
    float total_angle_ = 0.0;
    float speed_ = 0.0;

    IPwm *pwm_ = nullptr;
    IAngleSensor *angle_sensor_ = nullptr;
    ICurrentSensor *current_sensor_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    TaskHandle_t adc_task_handle_ = nullptr;
    SemaphoreHandle_t semaphore_ = nullptr;
    SemaphoreHandle_t adc_semaphore_ = nullptr;
};

#endif