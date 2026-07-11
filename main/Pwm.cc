#include "Pwm.h"
#include "esp_log.h"
#include "freeRTOS/freeRTOS.h"

static const char *TAG = "PWM";
Pwm::Pwm(int group_id, uint32_t freq_hz, int pinA, int pinB, int pinC)
{
	assert(pinA >= 0 && pinB >= 0 && pinC >= 0);
	group_id_ = group_id;
	freq_hz_ = freq_hz;
	pins_[0] = pinA;
	pins_[1] = pinB;
	pins_[2] = pinC;

	// ---------- 1. 创建定时器 ----------
	mcpwm_timer_config_t timer_cfg;
	memset(&timer_cfg, 0, sizeof(timer_cfg));
	timer_cfg.group_id = group_id_;
	timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
	timer_cfg.resolution_hz = resolution_hz_;
	timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;		 // 中央对齐
	timer_cfg.period_ticks = timer_cfg.resolution_hz / freq_hz_; // 一个周期的计数总数（不是峰值）
	timer_period_ = timer_cfg.period_ticks / 2;					 // 峰值用于占空比计算

	esp_err_t ret = mcpwm_new_timer(&timer_cfg, &timer_);
	ESP_ERROR_CHECK(ret);

	// ---------- 2. 创建三个通道的操作器、比较器、发生器 ----------
	for (int i = 0; i < 3; ++i)
	{
		// 操作器
		mcpwm_operator_config_t oper_cfg;
		memset(&oper_cfg, 0, sizeof(oper_cfg));
		oper_cfg.group_id = group_id_;
		ret = mcpwm_new_operator(&oper_cfg, &operators_[i]);
		ESP_ERROR_CHECK(ret);

		// 连接定时器
		ret = mcpwm_operator_connect_timer(operators_[i], timer_);
		ESP_ERROR_CHECK(ret);

		// 比较器（在计数器归零时更新）
		mcpwm_comparator_config_t cmp_cfg;
		memset(&cmp_cfg, 0, sizeof(cmp_cfg));
		cmp_cfg.flags.update_cmp_on_tez = true;
		ret = mcpwm_new_comparator(operators_[i], &cmp_cfg, &comparators_[i]);
		ESP_ERROR_CHECK(ret);

		// 发生器
		mcpwm_generator_config_t gen_cfg;
		memset(&gen_cfg, 0, sizeof(gen_cfg));
		gen_cfg.gen_gpio_num = pins_[i];
		ret = mcpwm_new_generator(operators_[i], &gen_cfg, &generators_[i]);
		// 比较动作：中央对齐 PWM (向上计数到比较值→高，向下计数到比较值→低)
		mcpwm_gen_compare_event_action_t cmp_actions[2] = {
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators_[i], MCPWM_GEN_ACTION_HIGH),
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators_[i], MCPWM_GEN_ACTION_LOW),
		};
		ret = mcpwm_generator_set_actions_on_compare_event(generators_[i],
														   cmp_actions[0], cmp_actions[1],
														   mcpwm_gen_compare_event_action_t{});
		ESP_ERROR_CHECK(ret);

		// 初始占空比设为 0
		mcpwm_comparator_set_compare_value(comparators_[i], 0);
	}

	// ---------- 3. 注册定时器事件回调（中断） ----------
	mcpwm_timer_event_callbacks_t cbs;
	memset(&cbs, 0, sizeof(cbs));
	cbs.on_full = onTimerFull; // 向上计数到周期时触发
	ret = mcpwm_timer_register_event_callbacks(timer_, &cbs, this);
	ESP_ERROR_CHECK(ret);

	// 在核心1上启动定时器
	xTaskCreatePinnedToCore([](void *p)
							{
								Pwm *self = static_cast<Pwm *>(p);
								mcpwm_timer_enable(self->timer_);
								vTaskDelete(NULL); },
							"enable", 2048, this, 5, NULL, 1);
	ESP_LOGI(TAG, "3-Phase PWM initialized @ %ld Hz", freq_hz_);
}

void Pwm::setCallback(void (*func)(void *), void *data)
{
	func_ = func;
	data_ = data;
}

void Pwm::start()
{
	mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
}

void Pwm::stop()
{
	mcpwm_timer_start_stop(timer_, MCPWM_TIMER_STOP_FULL);
}

void Pwm::setDuty(float dutyA, float dutyB, float dutyC)
{
	mcpwm_comparator_set_compare_value(comparators_[0], dutyToCompare(dutyA));
	mcpwm_comparator_set_compare_value(comparators_[1], dutyToCompare(dutyB));
	mcpwm_comparator_set_compare_value(comparators_[2], dutyToCompare(dutyC));
}

bool IRAM_ATTR Pwm::onTimerFull(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
	Pwm *self = static_cast<Pwm *>(user_ctx);
	if (!self || !self->func_)
		return false;

	self->func_(self->data_);
	return false;
}

uint32_t Pwm::dutyToCompare(float duty) const
{
	if (duty < 0.0f)
		duty = 0.0f;
	if (duty > 1.0f)
		duty = 1.0f;
	return (uint32_t)((1.0f - duty) * timer_period_);
}