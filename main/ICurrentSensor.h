#ifndef _icurrentsensor_h_
#define _icurrentsensor_h_

class ICurrentSensor
{
public:
    virtual ~ICurrentSensor() {}
    virtual bool getCurrent(float &a, float &b, float &c) = 0;
    virtual bool start() = 0;
};

#endif