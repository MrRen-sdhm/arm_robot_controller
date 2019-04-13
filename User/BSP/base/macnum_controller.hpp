#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Geometry>

#include "c620_motor.hpp"
#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"
#include "base_motor.hpp"

namespace hustac {

class PIDController {
public:
    float kp_;
    float ki_;
    float kd_;
    float i_max_;
    float i_min_;
    
    uint64_t last_update_ = 0;
    float interval_ = 0;
    float p_error_ = 0;
    float last_p_error_ = 0;
    float i_error_ = 0;
    float d_error_ = 0;
    float error_ = 0;
    float p_ = 0;
    float i_ = 0;
    float d_ = 0;
    float output_ = 0;
    
    PIDController(float kp, float ki = 0, float kd = 0, float i_min = 0, float i_max = 0)
        : kp_(kp), ki_(ki), kd_(kd), i_min_(i_min), i_max_(i_max) {
    }
    
    void reset() {
        last_update_ = 0;
        interval_ = 0;
        p_error_ = 0;
        last_p_error_ = 0;
        i_error_ = 0;
        d_error_ = 0;
        error_ = 0;
        p_ = 0;
        i_ = 0;
        d_ = 0;
        output_ = 0;
    }
    
    float update(float error, float interval = -1) {
        uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
        
        if (last_update_ == 0) {
            // first run
            error_ = error;
            
            p_error_ = error;
            
            if (interval > 0) {
                interval_ = interval;
                i_error_ = p_error_ * interval_;
                i_error_ = std::min(i_max_, i_error_);
                i_error_ = std::max(i_min_, i_error_);
            } else {
                interval_ = 0;
                i_error_ = 0;
            }
            
            d_error_ = 0;
        } else {
            if (interval > 0) {
                interval_ = interval;
            } else {
                interval_ = (now - last_update_) / 1e9f;
            }
            error_ = error;
            
            p_error_ = error;
            
            i_error_ += p_error_ * interval_;
            i_error_ = std::min(i_max_, i_error_);
            i_error_ = std::max(i_min_, i_error_);
            
            d_error_ = (p_error_ - last_p_error_) / interval_;
        }
        
        p_ = p_error_ * kp_;
        i_ = i_error_ * ki_;
        d_ = d_error_ * kd_;
        
        output_ = p_ + i_ + d_;
        
        last_update_ = now;
        last_p_error_ = p_error_;
        return output_;
    }
};

struct Twist2D {
    float linear_x = 0;
    float linear_y = 0;
    float angular = 0;
    
    Twist2D operator-(const Twist2D& rhs) const {
        Twist2D result;
        result.linear_x = linear_x - rhs.linear_x;
        result.linear_y = linear_y - rhs.linear_y;
        result.angular = angular - rhs.angular;
        return result;
    }
//    // 数乘
//    Twist2D operator*(const float &rhs) const {
//        Twist2D result;
//        result.linear_x = linear_x * rhs;
//        result.linear_y = linear_y * rhs;
//        result.angular = angular * rhs;
//        return result;
//    }
//    // 矩阵乘法
//    Twist2D operator*(const Twist2D &rhs) const {
//        float cos_w = std::cos(rhs.angular + angular / 2);
//        float sin_w = std::sin(rhs.angular + angular / 2);
//
//        Twist2D result;
//        result.linear_x = rhs.linear_x + linear_x * cos_w - linear_y * sin_w;
//        result.linear_y = rhs.linear_y + linear_x * sin_w + linear_y * cos_w;
//        result.angular = rhs.angular + angular;
//        return result;
//    }

    bool legal() const {
        return std::isfinite(linear_x) && std::isfinite(linear_y) && std::isfinite(angular);
    }
    
    void limit(const Twist2D& max) {
        linear_x = std::max(std::min(linear_x, max.linear_x), -max.linear_x);
        linear_y = std::max(std::min(linear_y, max.linear_y), -max.linear_y);
        angular = std::max(std::min(angular, max.angular), -max.angular);
    }
};

// 轮式里程计
class WheelOdometry {
public:
    Eigen::Affine2f pose_ = Eigen::Affine2f(Eigen::Matrix3f::Identity());
    float constraint_error_ = 0;
    
    float x() const {
        return pose_.translation().x();
    }
    float y() const {
        return pose_.translation().y();
    }
    float angle() const {
        return Eigen::Rotation2Df(pose_.linear()).angle();
    }
    float constraint_error() const {
        return constraint_error_;
    }
    const Eigen::Affine2f& pose() const {
        return pose_;
    }
    int pose(const Eigen::Affine2f& pose) {
        pose_ = pose;
        return 0;
    }
    void update(const Eigen::Vector2f& linear_velocity, float angular_velocity, float interval, float constraint_error_vel = 0) {
        pose_.rotate(Eigen::Rotation2Df(angular_velocity * interval).toRotationMatrix());
        pose_.translate(linear_velocity * interval);
        constraint_error_ += constraint_error_vel * interval;
//        Eigen::Affine2f movement(Eigen::Rotation2Df(angular_velocity * interval));
//        movement.translation() = linear_velocity * interval;
//        pose_ = movement * pose_;
    }
};

// 麦克纳姆轮移动底盘控制器
// Todo: 1. 车体重心的估计 -> 估计每轮最大静摩擦力 ->
// Todo:    避免每轮出力超过最大静摩擦力, 并保持合力方向正确, 对输出转矩进行限幅, 通过极限平衡的转矩控制达到精确可靠的速度闭环
// Todo: 2. 结合IMU, 对当前速度进行准确的估计 -> 避免由于某轮打滑, 导致速度估计错误, 进而导致错误的速度闭环
class MacnumController : virtual public EmergencyStoppable {
public:
    static constexpr float radius_wheel_ = 0.076f; // 轮子半径
    static constexpr float radius_left_right_ = 0.24f; // 左右轮距离/2
    static constexpr float radius_front_back_ = 0.26f; // 前后轮距离/2
    static constexpr Twist2D max_velocity_ = {.linear_x = 1.0f, .linear_y = 1.0f, .angular = float(M_PI) / 2};
    
    const char* name_;
    C620Motor& motor_;
    WheelOdometry odometry_;
    
    uint32_t count_update_ = 0;
    
    Twist2D desired_velocity_;
    Twist2D actual_velocity_;
    Twist2D error_velocity_;
    
    float constraint_error_velocity_ = 0;   // 运动约束误差 m/s
    
    Twist2D output_;
    
    PIDController pid_x_ = PIDController(12000, 60000, 0, -0.2f, 0.2f);
    PIDController pid_y_ = PIDController(12000, 60000, 0, -0.2f, 0.2f);
    PIDController pid_w_ = PIDController(6000, 30000, 0, -0.1f, 0.1f);
    
    MacnumController(const char* name, C620Motor& motor)
        : name_(name), motor_(motor) {
    }
    
    void on_emergency_stop_changed(bool value) override {
        if (!value) {
            pid_x_.reset();
            pid_y_.reset();
            pid_w_.reset();
        }
    }
    
    int desired_velocity(const Twist2D& twist) {
        if (!twist.legal()) {
            return -1;
        }
        auto vel = twist;
        vel.limit(max_velocity_);
        desired_velocity_ = vel;
        return 0;
    }
    
    const Twist2D& desired_velocity() {
        return desired_velocity_;
    }
    
    const Twist2D& actual_velocity() {
        return actual_velocity_;
    }
    
    WheelOdometry& odometry() {
        return odometry_;
    }
    
    //    uint32_t last_print = 0;
    
    void spin_once() {
        if (motor_.read() == 0) {
            count_update_++;
            
            update_actual_velocity();
            
            float interval = (motor_.interval_ > 0) ? motor_.interval_ : 0.001f;
            
            odometry_.update({actual_velocity_.linear_x, actual_velocity_.linear_y}, actual_velocity_.angular,
                             interval, constraint_error_velocity_);
            
            error_velocity_ = desired_velocity_ - actual_velocity_;
            
            //            if (HAL_GetTick() - last_print >= 500) {
            //                last_print += 500;
            //                printf("  : x=%.2f y=%.2f w=%.2f\n", actual_velocity_.linear_x, actual_velocity_.linear_y, actual_velocity_.angular / M_PI * 180);
            //                auto vel = inverse_kinematic(actual_velocity_);
            //                auto twist_4 = kinematic(vel);
            //                auto twist_LF = kinematic_LF(vel);
            //                auto twist_LB = kinematic_LB(vel);
            //                auto twist_RB = kinematic_RB(vel);
            //                auto twist_RF = kinematic_RF(vel);
            //                printf(" 4: x=%.2f y=%.2f w=%.2f\n", twist_4.linear_x, twist_4.linear_y, twist_4.angular / M_PI * 180);
            //                printf("LF: x=%.2f y=%.2f w=%.2f\n", twist_LF.linear_x, twist_LF.linear_y, twist_LF.angular / M_PI * 180);
            //                printf("LB: x=%.2f y=%.2f w=%.2f\n", twist_LB.linear_x, twist_LB.linear_y, twist_LB.angular / M_PI * 180);
            //                printf("RB: x=%.2f y=%.2f w=%.2f\n", twist_RB.linear_x, twist_RB.linear_y, twist_RB.angular / M_PI * 180);
            //                printf("RF: x=%.2f y=%.2f w=%.2f\n", twist_RF.linear_x, twist_RF.linear_y, twist_RF.angular / M_PI * 180);
            //            }
            
            output_.linear_x = pid_x_.update(error_velocity_.linear_x, interval);
            output_.linear_y = pid_y_.update(error_velocity_.linear_y, interval);
            output_.angular = pid_w_.update(error_velocity_.angular, interval);
            
            std::array<float, 4> desired_force = inverse_dynamics(output_);
            
            //            if (HAL_GetTick() - last_print >= 100) {
            //                last_print += 100;
            //                printf("%s: err={.x=%.2f,.y=%.2f,.w=%.2f}\n", name_, error_velocity_.linear_x, error_velocity_.linear_y, error_velocity_.angular);
            //                printf("%s: eff={.x=%.2f,.y=%.2f,.w=%.2f}\n", name_, output_.linear_x, output_.linear_y, output_.angular);
            //                printf("%s: eff={.LF=%.2f,.LB=%.2f,.RB=%.2f,.RF=%.2f}\n", name_, desired_effort[0], desired_effort[1], desired_effort[2], desired_effort[3]);
            //            }
            
            if (emergency_stop()) {
                for (int i = 0; i < 4; i++) {
                    motor_[i].desired_effort = 0;
                }
            } else {
                for (int i = 0; i < 4; i++) {
                    motor_[i].desired_effort = desired_force[i] * radius_wheel_;
                }
            }
            //           printf("%s: LF=%.2f deg/s, %f Nm, %f A\n", name_, motor_[0].velocity / float(M_PI) * 180, motor_[0].desired_effort, motor_[0].current);
            //            motor_.get_feedback(C620Motor::Motor::LEFT_FRONT).print("LEFT_FRONT: ");
            //            motor_.get_feedback(C620Motor::Motor::LEFT_BACK).print("LEFT_BACK: ");
            //            motor_.get_feedback(C620Motor::Motor::RIGHT_FRONT).print("RIGHT_FRONT: ");
            //            motor_.get_feedback(C620Motor::Motor::RIGHT_BACK).print("RIGHT_BACK: ");
            
            motor_.write();
        }
    }

private:
    // 正动力学: 车轮力矩->移动底盘力矩
    // [  1,          1,          1,         1         ]
    // [ -1,          1,         -1,         1         ]
    // [ -sqrt(2)*R, -sqrt(2)*R,  sqrt(2)*R, sqrt(2)*R ]
    Twist2D dynamics(const std::array<float, 4>& wheel_force) {
        const float& LF = wheel_force[0];
        const float& LB = wheel_force[1];
        const float& RB = wheel_force[2];
        const float& RF = wheel_force[3];
        Twist2D twist;
        float R = sqrtf(radius_left_right_ * radius_left_right_ + radius_front_back_ * radius_front_back_);
    
        twist.linear_x = LF + LB + RB + RF;
        twist.linear_y = -LF + LB - RB + RF;
        twist.angular = (-LF - LB + RB + RF) * sqrtf(2) * R;
        return twist;
    }
    // 逆动力学: 移动底盘力矩->车轮力矩
    // [ 1/4, -1/4, -sqrt(2)/(8*R) ]
    // [ 1/4,  1/4, -sqrt(2)/(8*R) ]
    // [ 1/4, -1/4,  sqrt(2)/(8*R) ]
    // [ 1/4,  1/4,  sqrt(2)/(8*R) ]
    // R = sqrt(a^2+b^2)
    std::array<float, 4> inverse_dynamics(const Twist2D& force) {
        std::array<float, 4> wheel_force;
        float& LF = wheel_force[0];
        float& LB = wheel_force[1];
        float& RB = wheel_force[2];
        float& RF = wheel_force[3];
        const float& x = force.linear_x;
        const float& y = force.linear_y;
        const float& w = force.angular;
        float R = sqrtf(radius_left_right_ * radius_left_right_ + radius_front_back_ * radius_front_back_);
        float k = sqrtf(2) / (8 * R);   // 0.5
        
        LF = x / 4 - y / 4 - k * w;
        LB = x / 4 + y / 4 - k * w;
        RB = x / 4 - y / 4 + k * w;
        RF = x / 4 + y / 4 + k * w;
        return wheel_force;
    }
    // 逆运动学: 移动底盘速度->车轮线速度
    // [ 1, -1, -r]
    // [ 1,  1, -r]
    // [ 1, -1,  r]
    // [ 1,  1,  r]
    std::array<float, 4> inverse_kinematic(const Twist2D& twist) {
        std::array<float, 4> vel;
        float& LF = vel[0];
        float& LB = vel[1];
        float& RB = vel[2];
        float& RF = vel[3];
        
        LF = twist.linear_x - twist.linear_y - twist.angular * (radius_left_right_ + radius_front_back_);
        LB = twist.linear_x + twist.linear_y - twist.angular * (radius_left_right_ + radius_front_back_);
        RB = twist.linear_x - twist.linear_y + twist.angular * (radius_left_right_ + radius_front_back_);
        RF = twist.linear_x + twist.linear_y + twist.angular * (radius_left_right_ + radius_front_back_);
        return vel;
    }
    // 运动约束误差: 车轮线速度->运动约束误差 m/s
    float kinematic_constraint(const std::array<float, 4>& vel) {
        const float& LF = vel[0];
        const float& LB = vel[1];
        const float& RB = vel[2];
        const float& RF = vel[3];
        
        return (LF + RF) - (LB + RB);
    }
    // 正运动学: 车轮线速度->移动底盘速度
    // [  1/4,  1/4,  1/4, 1/4 ]
    // [ -1/4,  1/4, -1/4, 1/4 ]
    // [ -r/4, -r/4,  r/4, r/4 ]
    Twist2D kinematic(const std::array<float, 4>& vel) {
        const float& LF = vel[0];
        const float& LB = vel[1];
        const float& RB = vel[2];
        const float& RF = vel[3];
        Twist2D twist;
        
        twist.linear_x = (LF + LB + RB + RF) / 4;
        twist.linear_y = (-LF + LB - RB + RF) / 4;
        twist.angular = (-LF - LB + RB + RF) / 4 / (radius_left_right_ + radius_front_back_);
        return twist;
    }
    // 忽略左前轮的正运动学
    // [      1/2, 1/2,       0]   [ LB ]
    // [        0, 1/2,    -1/2] * [ RB ]
    // [ -1/(2*r),   0, 1/(2*r)]   [ RF ]
    Twist2D kinematic_LF(const std::array<float, 4>& vel) {
        // const float& LF = vel[0];
        const float& LB = vel[1];
        const float& RB = vel[2];
        const float& RF = vel[3];
        Twist2D twist;
        
        twist.linear_x = (LB + RB) / 2;
        twist.linear_y = (RB - RF) / 2;
        twist.angular = (-LB + RF) / 2 / (radius_left_right_ + radius_front_back_);
        return twist;
    }
    // 忽略左后轮的正运动学
    // [      1/2,       0,  1/2]   [ LF ]
    // [        0,     1/2, -1/2] * [ RB ]
    // [ -1/(2*r), 1/(2*r),    0]   [ RF ]
    Twist2D kinematic_LB(const std::array<float, 4>& vel) {
        const float& LF = vel[0];
        // const float& LB = vel[1];
        const float& RB = vel[2];
        const float& RF = vel[3];
        Twist2D twist;
        
        twist.linear_x = (LF + RF) / 2;
        twist.linear_y = (RB - RF) / 2;
        twist.angular = (-LF + RB) / 2 / (radius_left_right_ + radius_front_back_);
        return twist;
    }
    // 忽略右后轮的正运动学
    // [ 1/2,        0,     1/2]   [ LF ]
    // [ 1/2,     -1/2,       0] * [ LB ]
    // [   0, -1/(2*r), 1/(2*r)]   [ RF ]
    Twist2D kinematic_RF(const std::array<float, 4>& vel) {
        const float& LF = vel[0];
        const float& LB = vel[1];
        // const float& RB = vel[2];
        const float& RF = vel[3];
        Twist2D twist;
        
        twist.linear_x = (LF + RF) / 2;
        twist.linear_y = (LF - LB) / 2;
        twist.angular = (-LB + RF) / 2 / (radius_left_right_ + radius_front_back_);
        return twist;
    }
    // 忽略右前轮的正运动学
    // [        0,  1/2,     1/2]   [ LF ]
    // [      1/2, -1/2,       0] * [ LB ]
    // [ -1/(2*r),    0, 1/(2*r)]   [ RB ]
    Twist2D kinematic_RB(const std::array<float, 4>& vel) {
        const float& LF = vel[0];
        const float& LB = vel[1];
        const float& RB = vel[2];
        // const float& RF = vel[3];
        Twist2D twist;
        
        twist.linear_x = (LB + RB) / 2;
        twist.linear_y = (LF - LB) / 2;
        twist.angular = (-LF + RB) / 2 / (radius_left_right_ + radius_front_back_);
        return twist;
    }
    
    // 估计当前底盘运行状态
    void update_actual_velocity() {
        // 计算车轮线速度, 0 1 2 3 对应 LF LB RB RF
        std::array<float, 4> vel;
        
        for (int i = 0; i < 4; i++) {
            vel[i] = motor_[i].velocity * radius_wheel_;
        }
        
        // 使用4个轮子转速求底盘运动速度
        actual_velocity_ = kinematic(vel);
        constraint_error_velocity_ = kinematic_constraint(vel);
    }
};

}
