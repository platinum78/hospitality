#ifndef PID_HPP_
#define PID_HPP_


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
    double error_, error_prev_, error_integral_, error_derivative_;
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
    error_integral_ += error_ * period_;
    error_derivative_ = (error_ - error_prev_) / period_;
    pidVal = error_ * kp_ + error_integral_ * ki_ + error_derivative_ * kd_;
    error_prev_ = error_;
    return pidVal;
}

#endif