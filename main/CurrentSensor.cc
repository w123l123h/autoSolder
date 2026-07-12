#include "CurrentSensor.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "CurrentSense";

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_CONV_FRAME_SIZE (4 * sizeof(adc_digi_output_data_t))  // 16 字节

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
	adc_continuous_handle_t handle = NULL;
	adc_continuous_handle_cfg_t adc_config = {
		.max_store_buf_size = 1024,
		.conv_frame_size = ADC_CONV_FRAME_SIZE,
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

	adc_continuous_config_t dig_cfg = {
		.sample_freq_hz = 200 * 1000,
		.conv_mode = EXAMPLE_ADC_CONV_MODE,
	};

	adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
	dig_cfg.pattern_num = channel_num;
	for (int i = 0; i < channel_num; i++)
	{
		adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
		adc_pattern[i].channel = channel[i] & 0x7;
		adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
		adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

		ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
		ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
		ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
	}
	dig_cfg.adc_pattern = adc_pattern;
	ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

	*out_handle = handle;
}

static bool adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
	// TODO: 实现 DMA 完成回调
	return false;
}

CurrentSensor::CurrentSensor(adc_channel_t adc_a, adc_channel_t adc_b)
	: adc_a_(adc_a), adc_b_(adc_b)
{
	adc_oneshot_unit_init_cfg_t adc_cfg = {};
	adc_cfg.unit_id = ADC_UNIT_1;
	adc_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle_));

	adc_oneshot_chan_cfg_t chan_cfg = {};
	chan_cfg.atten = ADC_ATTEN_DB_12;
	chan_cfg.bitwidth = ADC_BITWIDTH_12;
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, adc_a, &chan_cfg));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, adc_b, &chan_cfg));

	adc_cali_curve_fitting_config_t cali_cfg = {};
	cali_cfg.unit_id = ADC_UNIT_1;
	cali_cfg.atten = ADC_ATTEN_DB_12;
	cali_cfg.bitwidth = ADC_BITWIDTH_12;
	ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle_));

	ESP_LOGI(TAG, "ADC1 initialized, INA240 gain=%.0f, shunt=%.3f ohm", gain_, r_);
}

void CurrentSensor::adc_init()
{
	// 1. 创建句柄
	adc_continuous_handle_cfg_t handle_cfg = {};
	handle_cfg.max_store_buf_size = 1024;
	handle_cfg.conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 2; // 2 个通道

	ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_continuous_handle_));

	// 2. 配置转换格式与通道
	adc_digi_pattern_config_t pat[2] = {};
	pat[0].atten = ADC_ATTEN_DB_12;
	pat[0].channel = adc_a_;
	pat[0].unit = ADC_UNIT_1;
	pat[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
	pat[1].atten = ADC_ATTEN_DB_12;
	pat[1].channel = adc_b_;
	pat[1].unit = ADC_UNIT_1;
	pat[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

	adc_continuous_config_t dig_cfg = {};
	dig_cfg.pattern_num = 2;
	dig_cfg.adc_pattern = pat;
	dig_cfg.sample_freq_hz = 0; // 关键：由外部触发，不自动采样
	dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
	dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
	ESP_ERROR_CHECK(adc_continuous_config(adc_continuous_handle_, &dig_cfg));

	// 3. 注册 DMA 完成回调
	adc_continuous_evt_cbs_t cbs = {};
	cbs.on_conv_done = adc_conv_done_cb;
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_continuous_handle_, &cbs, NULL));
}

bool CurrentSensor::getCurrent(float &a, float &b, float &c)
{
	if (use_continuous_)
	{
		a = a_ / gain_ / r_ - 1650;
		b = b_ / gain_ / r_ - 1650;
		c = -a - b;
		return true;
	}
	int raw_a = 0, raw_b = 0;
	int mv_a = 0, mv_b = 0;

	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_a_, &raw_a));
	adc_cali_raw_to_voltage(cali_handle_, raw_a, &mv_a);

	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_b_, &raw_b));
	adc_cali_raw_to_voltage(cali_handle_, raw_b, &mv_b);

	float shunt_mv_a = (float)mv_a / gain_;
	float shunt_mv_b = (float)mv_b / gain_;
	a = shunt_mv_a / r_;
	b = shunt_mv_b / r_;
	a -= 1650;
	b -= 1650;
	c = -a - b;
	return true;
}

bool CurrentSensor::start()
{
	adc_continuous_handle_t handle = NULL;
	adc_channel_t channel[2] = {adc_a_, adc_b_};
	continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

	adc_continuous_evt_cbs_t cbs = {
		.on_conv_done = s_conv_done_cb,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
	ESP_ERROR_CHECK(adc_continuous_start(handle));

	use_continuous_ = true;
	return true;
}

bool IRAM_ATTR CurrentSensor::s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
	CurrentSensor *cur = (CurrentSensor *)user_data;
	assert(cur);

	adc_digi_output_data_t *p = (adc_digi_output_data_t *)edata->conv_frame_buffer;
	uint32_t num_samples = edata->size / sizeof(adc_digi_output_data_t);

	int u = 0, v = 0;
	bool got_u = false, got_v = false;

	for (uint32_t i = 0; i < num_samples; i++)
	{
		if (p[i].type2.channel == cur->adc_a_)
		{
			u = p[i].type2.data;
			got_u = true;
		}
		else if (p[i].type2.channel == cur->adc_b_)
		{
			v = p[i].type2.data;
			got_v = true;
		}

		// 一旦 U 和 V 都收集到，就更新全局变量
		if (got_u && got_v)
		{
			cur->a_ = u;
			cur->b_ = v;
			got_u = got_v = false; // 准备下一组
		}
	}
	return false;
}
