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
    void checkRecycled();

    static void timerCallback(TimerHandle_t timer);
    static void task(void *);

private:
    DRV8313 drv_;
    Foc foc_;
    Touch touch_;
    volatile float changed_angle_ = 0.0f;
    float position_ = 0.0f;
    float offset_ = -0.2;    // 每次回收，都少收一些
    float target_position_ = 0.0f;
    volatile WorkState state_ = WorkState::Idle;
    TimerHandle_t timer_ = nullptr;
    TaskHandle_t task_ = nullptr;
};

#endif