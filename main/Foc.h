#ifndef _FOC_H
#define _FOC_H

#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "IAngleSensor.h"
#include "ICurrentSensor.h"
#include "IPwm.h"

class Foc
{
public:
    void init(int pair, float r, int kv, float dc);

    void print()const;

    float updateOffset(float d);
    void offset(float offset)
    {
        offset_ = offset;
    }
    void pid(float p, float i, float d)
    {
        kp_ = p;
        ki_ = i;
        kd_ = d;
    }

    void q(float q)
    {
        tq_ = q;
    }
    void d(float d)
    {
        td_ = d;
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

    float err_q_ = 0.0f;
    float err_d_ = 0.0f;
    volatile float kp_ = 0.0f;
    volatile float ki_ = 0.0f;
    volatile float kd_ = 0.0f;

    adc_oneshot_unit_handle_t adc_handle = nullptr;
    adc_cali_handle_t cali_handle = nullptr;

    volatile float current_ma_a_ = 0;
    volatile float current_ma_b_ = 0;
    volatile float current_ma_c_ = 0;
    float angle_ = 0.0;     // 电角度
    float angle_rad_ = 0.0; // 转子角度
    float last_angle_ = -1.0;
    float total_angle_ = 0.0;

    IPwm *pwm_ = nullptr;
    IAngleSensor *angle_sensor_ = nullptr;
    ICurrentSensor *current_sensor_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    TaskHandle_t adc_task_handle_ = nullptr;
    SemaphoreHandle_t semaphore_ = nullptr;
    SemaphoreHandle_t adc_semaphore_ = nullptr;
};

#endif