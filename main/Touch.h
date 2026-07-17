#ifndef _TOUCH_H_
#define _TOUCH_H_

#include "driver/touch_sensor.h"
#include <functional>

enum TouchEvent
{
    TouchEvent_Touch,
    TouchEvent_Release,
};

enum TouchState
{
    Idle,
    Touched,
    Released,
};

typedef std::function<void(TouchEvent)> TouchCallback;

class Touch
{
public:
    void init(touch_pad_t touch_pad, int threshold, int interval_ms);

    void run();

    void setCallback(TouchCallback callback)
    {
        callback_ = callback;
    }

private:
    void touch_calibrate_baseline(void);
    void task();

private:
    touch_pad_t touch_pad_;
    int threshold_ = 10;
    TouchCallback callback_;
    int baseline_value_ = 0; // 动态基准值
    TouchState state_ = Idle;
    int interval_ms_ = 100;
};

#endif