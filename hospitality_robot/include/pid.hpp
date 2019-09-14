#ifndef PID_HPP_
#define PID_HPP_

#include <ctime>

class PID
{
public:
    PID(double kp, double ki, double kd, double hz);

public:
    void GetTime();
    void SetTarget(double val);
    void SetOutput(double val);

public:
    double ComputePID();

private:
    double kp_, ki_, kd_, hz_, period_;
    double target_, output_;
    double error_, error_integral_, error_derivative_;
};

PID::PID(double kp, double ki, double kd, double hz)
    : kp_(kp), ki_(ki), kd_(kd), hz_(hz),
      target_(0), output_(0), 
      error_integral_(0), error_derivative_(0)
{
    period_ = 1 / hz_;
}

void PID::SetTarget(double val)
{
    target_ = val;
}

void PID::SetOutput(double val)
{
    output_ = val;
}

double PID::ComputePID()
{
    double pidVal = 0;
    error_ = target_ - output_;
}

#endif