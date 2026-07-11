#include "Foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/spi_master.h"
#include "math.h"
#include "esp_log.h"

static constexpr float squareRoot3 = 1.73205f;
static const char *TAG = "FOC";

void Foc::init(int pair, float r, int kv, float dc)
{
    pair_ = pair;
    r_ = r;
    kv_ = kv;
    dc_ = dc;
}

void Foc::print() const
{
    ESP_LOGI(TAG, "q: %f, d: %f, angle: %f, total angle: %f, ma: %f, mb: %f, mc: %f",
             tq_, td_, angle_, total_angle_, current_ma_a_, current_ma_b_, current_ma_c_);
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
    xTaskCreatePinnedToCore(adc_task, "AdcTask", 4096, this,
                            configMAX_PRIORITIES - 1, &adc_task_handle_, 0);

    pwm_->start();
}

void Foc::stop()
{
    if (task_handle_)
    {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    if(adc_task_handle_)
    {
        vTaskDelete(adc_task_handle_);
        adc_task_handle_ = nullptr;
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
    pwm_->setAdcCallback(adc_func, this);
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
    angle_ = pair_ * angle_rad_ + M_PI * 0.5;
    angle_ = fmod(angle_, 2.0 * M_PI);
    if (angle_ < 0)
    {
        angle_ += 2.0 * M_PI;
    }

    update_duty();
}

void Foc::update_duty()
{
    q_ = tq_;
    d_ = td_;
    float Valpha = d_ * cosf(angle_) - q_ * sinf(angle_);
    float Vbeta = q_ * cosf(angle_) + d_ * sinf(angle_);
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
}
