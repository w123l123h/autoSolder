#include "Foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/spi_master.h"
#include "math.h"
#include "esp_log.h"
#include "esp_timer.h"

static constexpr float squareRoot3 = 1.73205f;
static const char *TAG = "FOC";

// 头文件或全局
#define SIN_TABLE_SIZE 1024
static float sin_table[SIN_TABLE_SIZE];

// 初始化时生成
void init_sin_table()
{
    for (int i = 0; i < SIN_TABLE_SIZE; i++)
    {
        sin_table[i] = sinf(2 * M_PI * i / SIN_TABLE_SIZE);
    }
}

// 快速 sine
inline float fast_sin(float angle)
{
    int idx = (int)(angle * (SIN_TABLE_SIZE / (2.0f * M_PI))) & (SIN_TABLE_SIZE - 1);
    return sin_table[idx];
}
inline float fast_cos(float angle)
{
    return fast_sin(angle + M_PI_2);
}

void Foc::init(int pair, float r, int kv, float dc, int dir)
{
    pair_ = pair;
    r_ = r;
    kv_ = kv;
    dc_ = dc;
    dir_ = dir;
    dc_max_ = dc_ * 0.666667;
    assert(dir_ == 1 || dir_ == -1);
    init_sin_table();
}

void Foc::print() const
{
    ESP_LOGI(TAG, "q: %f, d: %f, angle: %f, total angle: %f, ma: %f, mb: %f, mc: %f, pid internal: %d,  speed: %f",
             q_, d_, angle_, total_angle_, current_ma_a_, current_ma_b_, current_ma_c_, pid_ts_internal_, speed_);
}

float Foc::updateOffset(float d)
{
    update_offset_ = true;
    float temp = tq_;
    tq_ = d;
    angle_ = M_3PI_4 * 2;
    start();
    vTaskDelay(pdMS_TO_TICKS(500));
    int count = 0;
    float last_angle = -1.f;
    while (true)
    {
        float angle = angle_sensor_->getAngle();
        if (fabs(angle - last_angle) < 0.1)
        {
            ++count;
            if (count > 20)
            {
                offset_ = angle;
                break;
            }
        }
        else
        {
            count = 0;
        }
        last_angle = angle;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    stop();
    update_offset_ = false;
    tq_ = temp;
    return offset_;
}

void Foc::start()
{
    if (task_handle_)
        return;

    semaphore_ = xSemaphoreCreateBinary();
    adc_semaphore_ = xSemaphoreCreateBinary();
    last_angle_ = -1.0;
    xTaskCreatePinnedToCore(foc_task, "FOCTask", 4096, this,
                            configMAX_PRIORITIES - 1, &task_handle_, 1);
    // xTaskCreatePinnedToCore(adc_task, "AdcTask", 4096, this,
    //                         configMAX_PRIORITIES - 1, &adc_task_handle_, 0);

    pwm_->start();
}

void Foc::stop()
{
    if (pwm_)
    {
        pwm_->stop();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    if (task_handle_)
    {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    if (adc_task_handle_)
    {
        vTaskDelete(adc_task_handle_);
        adc_task_handle_ = nullptr;
    }
    if (semaphore_)
    {
        vSemaphoreDelete(semaphore_);
        semaphore_ = nullptr;
    }
    if (adc_semaphore_)
    {
        vSemaphoreDelete(adc_semaphore_);
        adc_semaphore_ = nullptr;
    }
}

void Foc::connect(IAngleSensor *p)
{
    angle_sensor_ = p;
}

void Foc::connect(ICurrentSensor *p)
{
    current_sensor_ = p;
}

void Foc::connect(IPwm *p)
{
    pwm_ = p;
    pwm_->setCallback(pwm_func, this);
    // pwm_->setAdcCallback(adc_func, this);
}

void Foc::foc_task(void *p)
{
    Foc *foc = (Foc *)p;
    assert(foc);

    while (1)
    {
        assert(foc->semaphore_);
        if (xSemaphoreTake(foc->semaphore_, portMAX_DELAY) == pdTRUE)
        {
            foc->update();
        }
    }
}

void Foc::adc_task(void *p)
{
    Foc *foc = (Foc *)p;
    assert(foc);

    while (1)
    {
        assert(foc->adc_semaphore_);
        if (xSemaphoreTake(foc->adc_semaphore_, portMAX_DELAY) == pdTRUE)
        {
            // 读取电流
            assert(foc->current_sensor_);
            foc->current_sensor_->getCurrent((float &)foc->current_ma_a_, (float &)foc->current_ma_b_, (float &)foc->current_ma_c_);
        }
    }
}

void Foc::pwm_func(void *p)
{
    Foc *foc = (Foc *)p;
    assert(foc);
    if (!foc->semaphore_)
        return;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(foc->semaphore_, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Foc::adc_func(void *p)
{
    Foc *foc = (Foc *)p;
    assert(foc);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(foc->adc_semaphore_, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Foc::update()
{
    if (update_offset_)
    {
        angle_ += offset_step_;
        update_duty();
        return;
    }
    assert(angle_sensor_);
    // --- 读取 AS5048A 角度 ---
    angle_rad_ = angle_sensor_->getAngle();
    // 无效角度
    if (angle_rad_ < 0)
    {
        return;
    }
    // 第一次读取
    if (last_angle_ < 0)
    {
        last_angle_ = angle_rad_;
        return;
    }
    float delta = angle_rad_ - last_angle_;

    if (delta > M_PI)
    {
        delta -= 2.0f * M_PI;
    }
    else if (delta < -M_PI)
    {
        delta += 2.0f * M_PI;
    }

    total_angle_ += delta;
    last_angle_ = angle_rad_;
    last_angle_ = angle_rad_;

    // angle += 0.01;
    angle_ = (pair_ * angle_rad_ - offset_) + M_PI * 0.5;
    angle_ = fmod(angle_, 2.0 * M_PI);
    if (angle_ < 0)
    {
        angle_ += 2.0 * M_PI;
    }

    if (++pid_count_ == 40)
    {
        int64_t us = esp_timer_get_time();
        pid_ts_internal_ = us - pid_ts_;
        speed_ = (total_angle_ - pid_angle_) * 1000000 / (pid_ts_internal_);
        pid_angle_ = total_angle_;
        // pid控制
        q_ = dir_ * pid_speed_.update(speed_); // / (kv_ * M_PI * 2 / 60);
        q_ = std::max(std::min(dc_max_, q_), -dc_max_);
        d_ = std::max(std::min(dc_max_, d_), -dc_max_);
        pid_count_ = 0;
        pid_ts_ = us;
    }

    update_duty();

    // 读取电流
    // assert(current_sensor_);
    // assert(current_sensor_->getCurrent((float &)current_ma_a_, (float &)current_ma_b_, (float &)current_ma_c_));
}

void Foc::update_duty()
{
    float Valpha = d_ * fast_cos(angle_) - q_ * fast_sin(angle_);
    float Vbeta = q_ * fast_cos(angle_) + d_ * fast_sin(angle_);

    float dutyA = 0;
    float dutyB = 0;
    float dutyC = 0;

    // 判断扇区
    int bit = (Vbeta > 0) | (((squareRoot3 * Valpha - Vbeta) > 0) << 1) | (((-squareRoot3 * Valpha - Vbeta) > 0) << 2);
    float t1, t2;
    float x = squareRoot3 * Vbeta / dc_;
    float y = squareRoot3 / dc_ * (squareRoot3 * 0.5 * Valpha + 0.5 * Vbeta);
    float z = squareRoot3 / dc_ * (-squareRoot3 * 0.5 * Valpha + 0.5 * Vbeta);
    switch (bit)
    {
    case 3:
        t1 = -z;
        t2 = x;
        dutyA = 0.5 * t1 + 0.5 * t2 + 0.5;
        dutyB = 0.5 * t2 + 0.5 - 0.5 * t1;
        dutyC = 0.5 - 0.5 * t1 - 0.5 * t2;
        break;
    case 1:
        t1 = z;
        t2 = y;
        dutyA = 0.5 * t2 + 0.5 - 0.5 * t1;
        dutyB = 0.5 * t1 + 0.5 * t2 + 0.5;
        dutyC = 0.5 - 0.5 * t1 - 0.5 * t2;
        break;
    case 5:
        t1 = x;
        t2 = -y;
        dutyA = 0.5 - 0.5 * t1 - 0.5 * t2;
        dutyB = 0.5 * t1 + 0.5 * t2 + 0.5;
        dutyC = 0.5 * t2 + 0.5 - 0.5 * t1;
        break;
    case 4:
        t1 = -x;
        t2 = z;
        dutyA = 0.5 - 0.5 * t1 - 0.5 * t2;
        dutyB = 0.5 * t2 + 0.5 - 0.5 * t1;
        dutyC = 0.5 * t1 + 0.5 * t2 + 0.5;
        break;
    case 6:
        t1 = -y;
        t2 = -z;
        dutyA = 0.5 * t2 + 0.5 - 0.5 * t1;
        dutyB = 0.5 - 0.5 * t1 - 0.5 * t2;
        dutyC = 0.5 * t1 + 0.5 * t2 + 0.5;
        break;
    case 2:
        t1 = y;
        t2 = -x;
        dutyA = 0.5 * t1 + 0.5 * t2 + 0.5;
        dutyB = 0.5 - 0.5 * t1 - 0.5 * t2;
        dutyC = 0.5 * t2 + 0.5 - 0.5 * t1;
        break;
    default:
        dutyA = 0.5;
        dutyB = 0.5;
        dutyC = 0.5;
    }

    assert(pwm_);
    pwm_->setDuty(dutyA, dutyB, dutyC);

    // if (update_offset_ && fabs(Vbeta) < 0.000001)
    // {
    //     offset_step_ = 0;
    //     // ESP_LOGI(TAG, "Vbeta == %f", Vbeta);
    // }
}
