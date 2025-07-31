#pragma once
#include <algorithm>

class PID {
public:
    PID(double kp, double ki, double kd, double i_max = 1.0)
        : kp_(kp), ki_(ki), kd_(kd), i_max_(i_max),
          prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -i_max_, i_max_);
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_, i_max_;
    double prev_error_;
    double integral_;
};
