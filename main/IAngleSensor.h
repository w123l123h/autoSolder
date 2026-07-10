#ifndef _IANGLE_SENSOR_H_
#define _IANGLE_SENSOR_H_

class IAngleSensor
{
public:
    virtual ~IAngleSensor() {}
    virtual float getAngle() = 0;
};

#endif