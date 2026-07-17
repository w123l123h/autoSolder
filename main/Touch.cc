#include "Touch.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "Touch"
void Touch::init(touch_pad_t touch_pad, int threshold, int interval_ms)
{
    ESP_LOGI(TAG, "初始化触摸传感器...");
    touch_pad_ = touch_pad;
    threshold_ = threshold;
    interval_ms_ = interval_ms;

    // 1. 初始化触摸传感器驱动
    ESP_ERROR_CHECK(touch_pad_init());

    // 2. 配置触摸引脚 (v5.x 只需一个参数)
    ESP_ERROR_CHECK(touch_pad_config(touch_pad_));

    // 3. 设置有限状态机为定时器模式，硬件自动采样
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));

    // 4. 启动 FSM
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    // 5. 配置滤波器，增加数据稳定性
    touch_filter_config_t filter_config = {
        .mode = TOUCH_PAD_FILTER_IIR_8,
        .debounce_cnt = 1,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2};
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_config));
    ESP_ERROR_CHECK(touch_pad_filter_enable());
    ESP_LOGI(TAG, "触摸传感器初始化完成，正在校准基准值...");

    touch_calibrate_baseline();
}

void Touch::run()
{
    xTaskCreatePinnedToCore([](void *pvParameters)
                            { 
                                Touch *touch = (Touch *)pvParameters; 
                                assert(touch);
                                touch->task(); },
                            "Touch", 4096, this, 5, NULL, 0);
}

void Touch::touch_calibrate_baseline(void)
{
    uint32_t raw = 0;
    const int samples = 10;
    uint32_t sum = 0;

    for (int i = 0; i < samples; i++)
    {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(touch_pad_, &raw));
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    baseline_value_ = sum / samples;
    ESP_LOGI(TAG, "基准值校准完毕: %d", baseline_value_);
}

void Touch::task()
{
    bool touch = false;
    int count = 0;
    while (true)
    {
        uint32_t raw = 0;
        ESP_ERROR_CHECK(touch_pad_read_raw_data(touch_pad_, &raw));
        bool cur_touch = raw > baseline_value_ * 2;
        if (touch != cur_touch)
        {
            count = 0;
        }
        ++count;
        if (count == threshold_)
        {
            if (state_ == Idle)
            {
                if (!cur_touch)
                    continue;
            }
            if ((state_ == Touched && cur_touch) || (state_ == Released && !cur_touch))
            {
                continue;
            }
            state_ = cur_touch ? Touched : Released;
            callback_(cur_touch ? TouchEvent_Touch : TouchEvent_Release);
        }
        touch = cur_touch;
        // ESP_LOGI(TAG, "触摸传感器值: %d", raw);
        vTaskDelay(pdMS_TO_TICKS(interval_ms_));
    }
}
