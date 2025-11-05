#include <iostream>
#ifdef __RASPBERRY_PI__  // Pi条件编译
#include <wiringPi.h>
#elif __JETSON__  // Jetson条件编译（需定义宏）
#include <jetson-gpio.h>  // 假设Jetson GPIO库
#endif
#include "pid_controller.hpp"  // 新增PID

class Motor {
public:
    Motor() {
        speed_ = 0;
#ifdef __RASPBERRY_PI__
        wiringPiSetup();  // Pi初始化GPIO
        pinMode(0, OUTPUT);  // 模拟PWM引脚
#elif __JETSON__
        gpioInitialise();  // Jetson初始化
#endif
        pid_.init(1.0, 0.1, 0.05, 100.0);  // PID初始（Kp, Ki, Kd, 输出限幅）
        std::cout << "Motor initialized (Hardware: " << (defined(__RASPBERRY_PI__) ? "Pi" : defined(__JETSON__) ? "Jetson" : "Sim") << ")\n";
    }
    ~Motor() {
#ifdef __RASPBERRY_PI__
        digitalWrite(0, LOW);  // 停止电机
#elif __JETSON__
        gpioWrite(0, LOW);
#endif
        std::cout << "Motor stopped and cleaned\n";
    }
    void setSpeed(int target_speed) {
        double output = pid_.compute(target_speed, getCurrentSpeed());  // PID平滑计算
        speed_ = static_cast<int>(output);
#ifdef __RASPBERRY_PI__
        pwmWrite(0, speed_);  // Pi PWM驱动轮子
#elif __JETSON__
        gpioPWM(0, speed_);  // Jetson PWM
#endif
        std::cout << "Target: " << target_speed << ", Adjusted: " << speed_ << "\n";
    }
    int getSpeed() const { return speed_; }
    int getCurrentSpeed() const { return speed_; }  // 模拟反馈（实际用编码器）

private:
    int speed_ = 0;
    PIDController pid_;  // PID实例
};

int main() {
    Motor motor;
    motor.setSpeed(80);  // 测试平滑控制
    std::cout << "Current speed: " << motor.getSpeed() << "\n";
    return 0;
}