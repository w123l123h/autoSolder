#ifndef _PID_H_
#define _PID_H_

class Pid
{
public:
    void t(float t)
    {
        t_ = t;
    }

    void pid(float kp, float ki, float kd)
    {
        enable_ = true;
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    float update(float c)
    {
        err_ = t_ - c;
        err_total_ += err_;
        float rt = kp_ * err_ + ki_ * err_total_ + kd_ * (err_ - err_last_);
        err_last_ = err_;
        return rt;
    }

private:
    bool enable_ = false;
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float err_ = 0.0f;
    float err_last_ = 0.0f;
    float err_total_ = 0.0f;
    float t_ = 0.0f;
};

#endif