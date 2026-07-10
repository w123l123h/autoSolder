#include "CurrentSensor.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char* TAG = "CurrentSense";

CurrentSensor::CurrentSensor(adc_channel_t adc_a, adc_channel_t adc_b)
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

bool CurrentSensor::getCurrent(float& a, float& b, float& c)
{
	int raw_a = 0, raw_b = 0;
	int mv_a = 0, mv_b = 0;

	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, ADC_CHANNEL_3, &raw_a));
	adc_cali_raw_to_voltage(cali_handle_, raw_a, &mv_a);

	ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, ADC_CHANNEL_5, &raw_b));
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

