#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    void init(double kp, double ki, double kd, double max_out) {
        kp_ = kp; ki_ = ki; kd_ = kd; max_out_ = max_out;
        integral_ = 0.0; prev_error_ = 0.0;
    }
    double compute(double setpoint, double current) {
        double error = setpoint - current;
        integral_ += error;  // 积分
        double derivative = error - prev_error_;  // 微分
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        output = std::clamp(output, -max_out_, max_out_);  // 限幅
        prev_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_, max_out_;
    double integral_, prev_error_;
};

#endif