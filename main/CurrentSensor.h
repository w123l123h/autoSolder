#ifndef _CURRENT_SENSOR_H_
#define _CURRENT_SENSOR_H_

#include "ICurrentSensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"

class CurrentSensor : public ICurrentSensor
{
public:
	CurrentSensor(adc_channel_t adc_a, adc_channel_t adc_b);

	void adc_init();

	virtual bool getCurrent(float &a, float &b, float &c) override;
	virtual bool start() override;

private:
	static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

private:
	volatile bool use_continuous_ = false;
	float gain_ = 50.0f;
	float r_ = 0.020f;
	adc_channel_t adc_a_;
	adc_channel_t adc_b_;

	int a_ = 0;
	int b_ = 0;

	adc_oneshot_unit_handle_t adc_handle_ = nullptr;
	adc_continuous_handle_t adc_continuous_handle_ = nullptr;
	adc_cali_handle_t cali_handle_ = nullptr;
};

#endif
