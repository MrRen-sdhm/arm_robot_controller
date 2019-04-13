#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>

#include "base_motor.hpp"
#include "dmabuffer_uart.hpp"
#include "stm32f4xx_hal.h"
#include "utils.hpp"

namespace hustac {

class PWMServo : public BaseMotor {
public:
    TIM_HandleTypeDef *htim_; // TIM定时器
    const uint32_t channel_;  // TIM通道
    
    const std::pair<float, float> range_ = std::make_pair(-float(M_PI), float(M_PI)); // 旋转范围, uint: rad
    const std::pair<int, int> range_pulse_width_ = std::make_pair(500, 2500);                 // 脉宽范围, unit: ms
public:
    PWMServo(const char *name, TIM_HandleTypeDef *htim, uint32_t channel, decltype(range_) range)
        : BaseMotor(name), htim_(htim), channel_(channel), range_(range) {}
    
    int init() override { return 0; }
    
    // 启动舵机信号输出
    int start() override {
        HAL_TIM_PWM_Start(htim_, channel_);
        return 0;
    }
    // 停止舵机信号输出
    int stop() override {
        HAL_TIM_PWM_Stop(htim_, channel_);
        return 0;
    }
    // 设定期望角度, unit: rad
    int goal_position(float rad) override {
        if (std::isfinite(rad)) {
            // 浮点数值正常
            rad = std::min(rad, range_.second);
            rad = std::max(rad, range_.first);
            int width = angle2width(rad);
            width = set_pulse_width(width);
            return 0;
        } else {
            // 浮点数值异常
            return -1;
        }
    }
    
    // 读取设定角度, unit: rad
    float goal_position() override {
        int width = (int) __HAL_TIM_GET_COMPARE(htim_, channel_);
        return width2angle(width);
    }
    
    float current_position() override {
        return goal_position();
    }

private:
    int angle2width(float rad) {
        // 转换到 [0, 1]
        float percent = (rad - range_.first) / (range_.second - range_.first);
        // 转换到脉宽
        int ms = (int) std::round(
            percent * (range_pulse_width_.second - range_pulse_width_.first) + range_pulse_width_.first);
        return ms;
    }
    float width2angle(int width) {
        // 转换到 [0, 1]
        float percent =
            (width - range_pulse_width_.first) / float(range_pulse_width_.second - range_pulse_width_.first);
        // 转换到 rad
        float rad = percent * (range_.second - range_.first) + range_.first;
        return rad;
    }
    int set_pulse_width(int ms) {
        // 限幅
        ms = std::min(ms, range_pulse_width_.second);
        ms = std::max(ms, range_pulse_width_.first);
        __HAL_TIM_SET_COMPARE(htim_, channel_, ms);
        return ms;
    }
};

// RML 轨迹跟踪, 开环!
class RMLController : public BaseMotor, virtual public EmergencyStoppable {
public:
    RMLController(const char *name, BaseMotor &motor, uint32_t interval_ms, float max_vel, float max_accel)
        : BaseMotor(name), motor_(motor), rml_(ReflexxesAPI(1, interval_ms / 1000.0f)),
          ip_(RMLPositionInputParameters(1)), op_(RMLPositionOutputParameters(1)) {
        ip_.MaxVelocityVector->VecData[0] = max_vel;   //15.0f / 180 * M_PI;
        ip_.MaxAccelerationVector->VecData[0] = max_accel; // 30.0f / 180 * M_PI;
        ip_.MaxJerkVector->VecData[0] = 1e9;       // 不限制加加速度
        ip_.SelectionVector->VecData[0] = true;
        // 初始状态为 0
        ip_.TargetPositionVector->VecData[0] = 0;
        ip_.TargetVelocityVector->VecData[0] = 0;
        timer_update_.set_interval(interval_ms);
    }
    
    int init() override {
        int ret = motor_.init();
        if (ret < 0) {
            return ret;
        }
//        float pos = 0;
//        ip_.CurrentPositionVector->VecData[0] = pos;
//        ip_.CurrentVelocityVector->VecData[0] = 0;
//        ip_.CurrentAccelerationVector->VecData[0] = 0;
//        op_.NewPositionVector->VecData[0] = pos;
//        op_.NewVelocityVector->VecData[0] = 0;
//        op_.NewAccelerationVector->VecData[0] = 0;
//        motor_.goal_position((float) ip_.CurrentPositionVector->VecData[0]);
        if (emergency_stop()) {
            motor_.stop();
        }
//        timer_update_.reset();
        return 0;
    }
    
    int start() override {
        if (emergency_stop()) {
            return -1;
        }
        if (!started) {
            started = true;
            float pos = motor_.current_position();
            if (!std::isfinite(pos) || pos == motor_.invalid_output) {
                return -2;
            }
            ip_.CurrentPositionVector->VecData[0] = pos;
            ip_.CurrentVelocityVector->VecData[0] = 0;
            ip_.CurrentAccelerationVector->VecData[0] = 0;
            op_.NewPositionVector->VecData[0] = pos;
            op_.NewVelocityVector->VecData[0] = 0;
            op_.NewAccelerationVector->VecData[0] = 0;
            motor_.goal_position((float) ip_.CurrentPositionVector->VecData[0]);
            motor_.start();
            timer_update_.reset();
            return 0;
        } else {
            return 1;
        }
    }
    int stop() override {
        if (started) {
            started = false;
            motor_.stop();
            return 0;
        } else {
            return 1;
        }
    }
    
    void on_emergency_stop_changed(bool val) override {
        if (val) {
            started = false;
            motor_.stop();
            printf("%s: emergency stop!\n", name_);
        }
    }
    
    // 读取目标位姿
    float goal_position() override {
        return (float) ip_.TargetPositionVector->VecData[0];
    }
    
    // 设定目标位姿
    int goal_position(float pos) override {
        float save_goal_pos = motor_.goal_position();
        int ret = motor_.goal_position(pos);
        if (ret < 0) {
            motor_.goal_position(save_goal_pos);
            return ret;
        }
        pos = motor_.goal_position();
        motor_.goal_position(save_goal_pos);

//        if (ip_.TargetPositionVector->VecData[0] != (double) pos) {
//            printf("%s: Goal=%.3f\n", name_, pos);
//        }
        ip_.TargetPositionVector->VecData[0] = pos;
        return 0;
    }
    
    // 读取当前位姿
    float current_position() override {
        return motor_.goal_position();
        return motor_.current_position();
    }
    
    // 读取当前速度
    float current_velocity() override {
        return motor_.current_velocity();
    }
    
    float current_effort() override {
        return motor_.current_effort();
    }
    
    // 读取当前加速度
    virtual float current_acceleration() {
        return (float) ip_.CurrentAccelerationVector->VecData[0];
    }
    
    int _update() {
        int ret = rml_.RMLPosition(ip_, &op_, flags_);
        if (ret < 0) {
            // 发生错误
            printf("%s: RML error %d\n", name_, ret);
        }
        ip_.CurrentPositionVector->VecData[0] = op_.NewPositionVector->VecData[0];
        ip_.CurrentVelocityVector->VecData[0] = op_.NewVelocityVector->VecData[0];
        ip_.CurrentAccelerationVector->VecData[0] = op_.NewAccelerationVector->VecData[0];
        return motor_.goal_position((float) op_.NewPositionVector->VecData[0]);
    }
    
    int spin_once() override {
        if (started && timer_update_.is_timeout()) {
            return _update();
        }
        motor_.spin_once();
        return 0;
    }

protected:
    BaseMotor &motor_;
    ReflexxesAPI rml_;
    RMLPositionInputParameters ip_;
    RMLPositionOutputParameters op_;
    RMLPositionFlags flags_;
    bool started = false;
    float last_pos = std::numeric_limits<float>::signaling_NaN();
    float last_vel = std::numeric_limits<float>::signaling_NaN();
    SoftTimerMS<> timer_update_ = SoftTimerMS<>(0);
};

// 多电机同步 RML 轨迹跟踪, 开环!
template <int motor_count>
class MultipleRMLController : virtual public EmergencyStoppable {
public:
    static const int motor_count_ = motor_count;
    
    MultipleRMLController(const char *name, std::array<BaseMotor *, motor_count_> &motors, uint32_t interval_ms,
                          std::array<float, motor_count_> max_vel, std::array<float, motor_count_> max_accel)
        : motors_(motors), rml_(ReflexxesAPI(motor_count_, interval_ms / 1000.0f)),
          ip_(RMLPositionInputParameters(motor_count_)), op_(RMLPositionOutputParameters(motor_count_)) {
        for (int i = 0; i < motor_count_; i++) {
            ip_.MaxVelocityVector->VecData[i] = max_vel[i];   //15.0f / 180 * M_PI;
            ip_.MaxAccelerationVector->VecData[i] = max_accel[i]; // 30.0f / 180 * M_PI;
            ip_.MaxJerkVector->VecData[i] = 1e9;       // 不限制加加速度
            ip_.SelectionVector->VecData[i] = true;
            // 初始状态为 0
            ip_.TargetPositionVector->VecData[i] = 0;
            ip_.TargetVelocityVector->VecData[i] = 0;
        }
        flags_.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
        timer_update_.set_interval(interval_ms);
    }
    
    int init() {
        for (BaseMotor *motor : motors_) {
            int ret = motor->init();
            if (ret < 0) {
                return ret;
            }
            if (emergency_stop()) {
                motor->stop();
            }
        }
        return 0;
    }
    
    int start() {
        if (emergency_stop()) {
            return -1;
        }
        if (!started) {
            started = true;
            for (int i = 0; i < motor_count_; i++) {
                float pos = motors_[i].current_position();
                if (!std::isfinite(pos) || pos == BaseMotor::invalid_output) {
                    return -2;
                }
                ip_.CurrentPositionVector->VecData[0] = pos;
                ip_.CurrentVelocityVector->VecData[0] = 0;
                ip_.CurrentAccelerationVector->VecData[0] = 0;
                op_.NewPositionVector->VecData[0] = pos;
                op_.NewVelocityVector->VecData[0] = 0;
                op_.NewAccelerationVector->VecData[0] = 0;
                motors_[i].goal_position((float) ip_.CurrentPositionVector->VecData[0]);
                motors_[i].start();
            }
            timer_update_.reset();
            return 0;
        } else {
            return 1;
        }
    }
    int stop() {
        if (started) {
            started = false;
            for (BaseMotor *motor : motors_) {
                motor->stop();
            }
            return 0;
        } else {
            return 1;
        }
    }
    
    void on_emergency_stop_changed(bool val) override {
        if (val) {
            started = false;
            for (BaseMotor *motor : motors_) {
                motor->stop();
            }
            printf("%s: emergency stop!\n", name_);
        }
    }
    
    // 读取目标位姿
    std::array<float, motor_count_> goal_position() {
        std::array<float, motor_count_> ret = {0};
        for (int i = 0; i < motor_count_; i++) {
            ret[i] = (float) ip_.TargetPositionVector->VecData[i];
        }
        return ret;
    }
    
    // 设定目标位姿
    int goal_position(std::array<float, motor_count_> pos) {
        for (int i = 0; i < motor_count_; i++) {
            BaseMotor *motor = motors_[i];
            float save_goal_pos = motor->goal_position();
            int ret = motor->goal_position(pos[i]);
            if (ret < 0) {
                motor->goal_position(save_goal_pos);
                return ret;
            }
            pos[i] = motor->goal_position();
            motor->goal_position(save_goal_pos);
            ip_.TargetPositionVector->VecData[0] = pos[i];
        }

//        if (ip_.TargetPositionVector->VecData[0] != (double) pos) {
//            printf("%s: Goal=%.3f\n", name_, pos);
//        }
        return 0;
    }
    
    // 读取当前位姿
    std::array<float, motor_count_> current_position() {
        std::array<float, motor_count_> ret = {0};
        for (int i = 0; i < motor_count_; i++) {
            ret[i] = motors_[i]->goal_position();
//            ret[i] = motors_[i]->current_position();
        }
        return ret;
    }
    
    // 读取当前速度
    std::array<float, motor_count_> current_velocity() {
        std::array<float, motor_count_> ret = {0};
        for (int i = 0; i < motor_count_; i++) {
            ret[i] = motors_[i]->current_velocity();
        }
        return ret;
    }
    
    std::array<float, motor_count_> current_effort() {
        std::array<float, motor_count_> ret = {0};
        for (int i = 0; i < motor_count_; i++) {
            ret[i] = motors_[i]->current_effort();
        }
        return ret;
    }
    
    // 读取当前加速度
    std::array<float, motor_count_> current_acceleration() {
        std::array<float, motor_count_> ret = {0};
        for (int i = 0; i < motor_count_; i++) {
            ret[i] = (float) ip_.CurrentAccelerationVector->VecData[i];
        }
        return ret;
    }
    
    int _update() {
        int ret = rml_.RMLPosition(ip_, &op_, flags_);
        if (ret < 0) {
            // 发生错误
            printf("%s: RML error %d\n", name_, ret);
        }
        for (int i = 0; i < motor_count_; i++) {
            ip_.CurrentPositionVector->VecData[i] = op_.NewPositionVector->VecData[i];
            ip_.CurrentVelocityVector->VecData[i] = op_.NewVelocityVector->VecData[i];
            ip_.CurrentAccelerationVector->VecData[i] = op_.NewAccelerationVector->VecData[i];
            motors_[i]->goal_position((float) op_.NewPositionVector->VecData[i]);
        }
        return ret;
    }
    
    int spin_once() {
        if (started && timer_update_.is_timeout()) {
            return _update();
        }
        for (int i = 0; i < motor_count_; i++) {
            motors_[i]->spin_once();
        }
        return 0;
    }

protected:
    const char *name_ = "";
    std::array<BaseMotor *, motor_count_> motors_;
    ReflexxesAPI rml_;
    RMLPositionInputParameters ip_;
    RMLPositionOutputParameters op_;
    RMLPositionFlags flags_;
    bool started = false;
    SoftTimerMS<> timer_update_ = SoftTimerMS<>(0);
};

}
