#ifndef _FOC_H
#define _FOC_H

#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "IAngleSensor.h"
#include "ICurrentSensor.h"
#include "IPwm.h"
#include "Pid.h"

class Foc
{
public:
    /**
     * @param dir 1:正转 -1:反转
     */
    void init(int pair, float r, int kv, float dc, int dir, int pid_interval);

    void print() const;

    float updateOffset(float d);
    void offset(float offset)
    {
        offset_ = offset;
    }
    void speedPid(float p, float i, float d, float max_i_v, float max_v)
    {
        pid_speed_.pid(p, i, d, max_i_v, max_v);
    }
    void resetSpeed()
    {
        pid_speed_.reset();
    }

    void positionPid(float p, float i, float d, float max_i_speed, float max_speed)
    {
        pid_position_.pid(p, i, d, max_i_speed, max_speed);
    }
    void resetPosition()
    {
        pid_position_.reset();
    }

    void speed(float speed)
    {
        pid_speed_.t(speed);
    }
    void position(float position)
    {
        pid_position_.t(position);
    }

    float position()const;

    float totalAngle() const
    {
        return total_angle_;
    }

    void init_angle();

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
    float dc_max_ = 0.0f;
    int dir_ = 1;

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
    int pid_interval_ = 1;
    float speed_max_ = 60.0f;

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
    float position_ = 0.0;

    IPwm *pwm_ = nullptr;
    IAngleSensor *angle_sensor_ = nullptr;
    ICurrentSensor *current_sensor_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    TaskHandle_t adc_task_handle_ = nullptr;
    SemaphoreHandle_t semaphore_ = nullptr;
    SemaphoreHandle_t adc_semaphore_ = nullptr;
};

#endif