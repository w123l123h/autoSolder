#ifndef _IPWM_H_
#define _IPWM_H_

#include <cstdint>

class IPwm
{
public:
    virtual ~IPwm() {}
    virtual void setCallback(void (*func)(void *), void *data) = 0;
    virtual void setAdcCallback(void (*func)(void *), void *data) = 0;
    virtual void start() = 0;
    virtual void setDuty(float dutyA, float dutyB, float dutyC) = 0;
};

#endif