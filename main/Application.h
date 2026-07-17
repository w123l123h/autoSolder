#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "Foc.h"
#include "Touch.h"
#include "DRV8313.h"

enum class WorkState
{
    Idle,
    Using,
    Waiting,
    Recycling,
};

class Application
{
public:
    Application();

    void init();

    void run();

private:
    void touched();
    void released();
    void recycled();
    static void timerCallback(TimerHandle_t timer);

private:
    DRV8313 drv_;
    Foc foc_;
    Touch touch_;
    volatile float changed_angle_ = 0.0f;
    float position_ = 0.0f;
    float traget_position_ = 0.0f;
    volatile WorkState state_ = WorkState::Idle;
    TimerHandle_t timer_ = nullptr;
};

#endif