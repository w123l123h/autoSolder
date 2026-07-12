#ifndef _PWM_H_
#define _PWM_H_

#include "IPwm.h"
#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"

class Pwm : public IPwm
{
public:
    Pwm(int group_id, uint32_t freq_hz, int pinA, int pinB, int pinC);

    virtual void setCallback(void (*func)(void *), void *data) override;
    virtual void setAdcCallback(void (*func)(void *), void *data) override;
    virtual void start() override;
    virtual void stop() override;
    virtual void setDuty(float dutyA, float dutyB, float dutyC) override;

private:
    static bool IRAM_ATTR onTimerFull(mcpwm_timer_handle_t timer,
                                      const mcpwm_timer_event_data_t *edata,
                                      void *user_ctx);
    static bool IRAM_ATTR cmp_on_reach_cb(mcpwm_cmpr_handle_t comparator,
                                          const mcpwm_compare_event_data_t *edata,
                                          void *user_ctx);
    uint32_t dutyToCompare(float duty) const;

private:
    uint32_t resolution_hz_ = 20000000; // 20 MHz
    int group_id_ = 0;
    uint32_t freq_hz_ = 0;
    int pins_[3] = {-1, -1, -1};
    uint32_t timer_period_ = 0;

    void (*func_)(void *) = nullptr;
    void *data_ = nullptr;
    void (*adc_func_)(void *) = nullptr;
    void *adc_data_ = nullptr;

    mcpwm_timer_handle_t timer_ = nullptr;
    mcpwm_oper_handle_t operators_[3] = {nullptr};
    mcpwm_cmpr_handle_t comparators_[3] = {nullptr};
    mcpwm_gen_handle_t generators_[3] = {nullptr};
    mcpwm_cmpr_handle_t adc_comparator_ = nullptr;
};

#endif
