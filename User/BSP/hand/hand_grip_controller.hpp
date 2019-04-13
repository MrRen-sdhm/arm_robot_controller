#pragma once

#include <cmath>

#include <control_msgs/GripperCommandAction.h>

#include "dmabuffer_uart.hpp"
#include "ocservo_rs485.hpp"
#include "utils.hpp"

namespace hustac {

class HandGripController : virtual public EmergencyStoppable {
public:
    const char* name_;
    OCServoRS485& hand_;

    // 夹取状态
    enum class State : uint8_t {
        WAIT_MOTOR,
        IDLE,
        STARTING,
        GRIPING,
        STALLED,
    };

    struct Parameter {
        // 关节位置限制, {0, 0}表示环形关节
        std::pair<float, float> joint_limits;
        // 堵转时的关节最大力矩限制, unit: Nm
        float effort_limits = 0.4f * (30 / 100.0f * 9.8); // 40% 最大力矩, 最大力矩为 2.94 Nm
        // 堵转检测阈值
        float stall_effort = 0.2f;
        // 位置容差
        float tolerance_goal_position    = 0.1f / 180 * float(M_PI);
        float tolerance_stalled_position = 1.0f / 180 * float(M_PI);

        volatile State state       = State::WAIT_MOTOR;
        volatile bool new_feedback = false;
        volatile bool new_result   = false;
        volatile bool enable_motor = false;
        control_msgs::GripperCommandGoal goal;
        control_msgs::GripperCommandFeedback feedback;
        control_msgs::GripperCommandResult result;

        static const uint32_t stalled_confirm_duration = 300; // 连续堵转超过该时间才判定为夹住, ms
        uint32_t stalled_time                          = 0; // 持续堵转开始的时间, ms
        float stalled_position                         = std::numeric_limits<float>::quiet_NaN(); // 堵转的位置
    };
    // 每个电机的参数
    Parameter param_[OCServoRS485::motor_count_];

    HandGripController(const char* name, OCServoRS485& hand);

    /* 由手爪驱动调用 (可能为中断) */

    // 需要写入控制信息时
    void on_sync_write();

    // 读取到新的电机数据时
    void on_read_sensor();

    /* 由ROS action server调用 */

    // 设定指定序号的夹爪的夹取操作
    // 若发生错误则返回值<0, 并可以通过get_result获取错误信息
    int set_goal(int id, const control_msgs::GripperCommandGoal& goal);

    // 取消当前夹取操作
    int cancel(int id);

    // 获取反馈信息
    int get_feedback(int id, control_msgs::GripperCommandFeedback& fb);

    // 在执行完毕/出错后, 返回值为0, 可以获取执行结果
    int get_result(int id, control_msgs::GripperCommandResult& result);
    
    void on_emergency_stop_changed(bool value) override;
};

/* 实现 */

inline HandGripController::HandGripController(const char* name, OCServoRS485& hand)
    : name_(name)
    , hand_(hand) {
    for (int i = 0; i < OCServoRS485::motor_count_; i++) {
        Parameter& param = param_[i];
        // 角度限制
        param.joint_limits = std::pair<float, float>(-180 * M_PI / 180, -118 * M_PI / 180);

        param.goal.command.position   = std::numeric_limits<float>::quiet_NaN();
        param.goal.command.max_effort = std::numeric_limits<float>::quiet_NaN();

        param.feedback.position     = std::numeric_limits<float>::quiet_NaN();
        param.feedback.effort       = std::numeric_limits<float>::quiet_NaN();
        param.feedback.stalled      = false;
        param.feedback.reached_goal = false;

        param.result.position     = std::numeric_limits<float>::quiet_NaN();
        param.result.effort       = std::numeric_limits<float>::quiet_NaN();
        param.result.stalled      = false;
        param.result.reached_goal = false;
    }

    hand_.set_read_pos_callback(std::bind(&HandGripController::on_read_sensor, this));
    hand_.set_sync_write_callback(std::bind(&HandGripController::on_sync_write, this));
}

inline void HandGripController::on_emergency_stop_changed(bool value) {
    if (value) {
        for (int i = 0; i < OCServoRS485::motor_count_; i++) {
            param_[i].enable_motor = false;
            hand_.enable(i + 1, param_[i].enable_motor);
            if (param_[i].state == State::STARTING
                || param_[i].state == State::GRIPING
                || param_[i].state == State::STALLED) {
                param_[i].state      = State::IDLE;
                param_[i].new_result = true;
            }
        }
        printf("%s: Emergency stop\n", name_);
    } else {
        printf("%s: Disable emergency stop\n", name_);
    }
}

inline void HandGripController::on_sync_write() {
    for (int i = 0; i < OCServoRS485::motor_count_; i++) {
        Parameter& param = param_[i];
        auto& motor      = hand_.motor(i + 1);

        if (param.state == State::WAIT_MOTOR) {
            // 尚未连接到电机
            param.enable_motor = false;
            continue;
        }

        if (param.state == State::STARTING) {
            // 写入目标位置信息
            param.enable_motor = true;
            motor.goal_position(param.goal.command.position);
            // 未堵转时, 不限制占空比
            if (hand_.set_max_load(i + 1, 1.0f) == 0) {
                // max_load有更新
                printf("%s[%d]: set_max_load(%.2f)\n", name_, i + 1, 1.0f);
            }

            param.stalled_time = 0;
            param.state        = State::GRIPING;
            printf("%s[%d]: GRIPING\n", name_, i + 1);
        }
    }
    for (int i = 0; i < OCServoRS485::motor_count_; i++) {
        hand_.enable(i + 1, emergency_stop() ? false : param_[i].enable_motor);
    }
}

inline void HandGripController::on_read_sensor() {
    for (int i = 0; i < OCServoRS485::motor_count_; i++) {
        Parameter& param = param_[i];
        auto& motor      = hand_.motor(i + 1);

        if (param.state == State::WAIT_MOTOR) {
            // 首次接收到电机数据, 记为已连接到电机
            param.state = State::IDLE;
            printf("%s[%d]: Connected to motor\n", name_, i + 1);
        }

        param.feedback.position = motor.current_position();
        param.feedback.effort   = motor.current_effort();
        param.result.position   = param.feedback.position;
        param.result.effort     = param.feedback.effort;

        // 检查是否到位
        float pos_err = motor.goal_position() - motor.current_position();
        if (std::abs(pos_err) <= param.tolerance_goal_position) {
            param.feedback.reached_goal = true;
            param.result.reached_goal   = true;
        } else {
            param.feedback.reached_goal = false;
            param.result.reached_goal   = false;
        }

        if (param.state == State::STARTING) {
            // 尚未写入目标位置
            param.stalled_time          = 0;
            param.feedback.stalled      = false;
            param.result.stalled        = false;
            param.feedback.reached_goal = false;
            param.result.reached_goal   = false;
        } else if (param.state == State::GRIPING) {
            // 检查是否堵转
            if (std::abs(param.feedback.effort) >= param.stall_effort) {
                if (param.stalled_time == 0) {
                    // 刚刚开始堵转
                    param.stalled_time     = HAL_GetTick();
                    param.feedback.stalled = false;
                    param.result.stalled   = false;
                } else if (HAL_GetTick() - param.stalled_time >= param.stalled_confirm_duration) {
                    // 堵转超过一定时间, 确认为抓住
                    param.feedback.stalled = true;
                    param.result.stalled   = true;
                } else {
                    // 连续堵转时间不够长, 继续等待
                    param.feedback.stalled = false;
                    param.result.stalled   = false;
                }
            } else {
                // 未堵转
                param.stalled_time     = 0;
                param.feedback.stalled = false;
                param.result.stalled   = false;
            }

            if (param.result.stalled) {
                // 夹取成功, 保持堵转, 设定电机最大占空比为max_effort对应值, 避免舵机损坏
                float max_load;
                if (std::isfinite(param.goal.command.max_effort)) {
                    max_load = param.goal.command.max_effort / (30 / 100.0f * 9.8);
                } else {
                    max_load = param.effort_limits / (30 / 100.0f * 9.8);
                }
                if (hand_.set_max_load(i + 1, max_load) == 0) {
                    // max_load有更新
                    printf("%s[%d]: set_max_load(%.2f)", name_, i + 1, max_load);
                }

                // 记录夹取的位置
                param.stalled_time     = 0;
                param.stalled_position = param.result.position;
                param.state            = State::STALLED;
                printf("%s[%d]: Grip success\n", name_, i + 1);
            } else if (param.result.reached_goal) {
                // 夹取失败, 未夹取到物体
                if (hand_.set_max_load(i + 1, 1.0f) == 0) {
                    // max_load有更新
                    printf("%s[%d]: set_max_load(%.2f)\n", name_, i + 1, 1.0f);
                }

                param.new_result = true;
                param.state      = State::IDLE;
                printf("%s[%d]: Grip nothing\n", name_, i + 1);
            }
        } else if (param.state == State::STALLED) {
            // 检查是否堵转
            if (std::abs(param.feedback.effort) < param.stall_effort) {
                if (param.stalled_time == 0) {
                    // 刚刚开始不堵转
                    param.stalled_time     = HAL_GetTick();
                    param.feedback.stalled = true;
                    param.result.stalled   = true;
                } else if (HAL_GetTick() - param.stalled_time >= param.stalled_confirm_duration) {
                    // 不堵转超过一定时间, 确认为丢失
                    param.feedback.stalled = false;
                    param.result.stalled   = false;
                } else {
                    // 连续不堵转时间不够长, 继续等待
                    param.feedback.stalled = true;
                    param.result.stalled   = true;
                }
            } else {
                // 堵转
                param.stalled_time     = 0;
                param.feedback.stalled = true;
                param.result.stalled   = true;
            }

            if (param.result.reached_goal) {
                // 夹取失败, 未夹取到物体
                if (hand_.set_max_load(i + 1, 1.0f) == 0) {
                    // max_load有更新
                    printf("%s[%d]: set_max_load(%.2f)", name_, i + 1, 1.0f);
                }

                param.new_result = true;
                param.state      = State::IDLE;
                printf("%s[%d]: Grip nothing\n", name_, i + 1);
            } else if (!param.result.stalled) {
                // 夹取到的物体丢失!!!
                if (hand_.set_max_load(i + 1, 1.0f) == 0) {
                    // max_load有更新
                    printf("%s[%d]: set_max_load(%.2f)", name_, i + 1, 1.0f);
                }

                // 继续夹取
                param.stalled_time = 0;
                param.state        = State::GRIPING;
                printf("%s[%d]: Griped object lost!!!\n", name_, i + 1);
            }
        }

        param.new_feedback = true;
    }
}

inline int HandGripController::set_goal(int id, const control_msgs::GripperCommandGoal& goal) {
    // 检查目标是否合法
    if (id < 1 || id > OCServoRS485::motor_count_) {
        // 错误的手爪id
        printf("%s: Rejected, bad hand id %d\n", name_, id);
        return -1;
    }
    Parameter& param = param_[id - 1];
    if (std::isnan(goal.command.position) || std::isnan(goal.command.max_effort)) {
        // 若目标位置或最大力矩为NaN, 则禁用电机
        param.enable_motor = false;
        printf("%s[%d]: Motor disabled\n", name_, id);
    }
    if (!std::isfinite(goal.command.position) || goal.command.position < param.joint_limits.first || goal.command.position > param.joint_limits.second) {
        // 非法的目标位置
        param.result.stalled      = false;
        param.result.reached_goal = false;
        param.new_result          = true;
        printf("%s[%d]: Rejected, bad goal position %.2f [%.2f, %.2f]\n", name_, id, float(goal.command.position / M_PI * 180), float(param.joint_limits.first / M_PI * 180), float(param.joint_limits.second / M_PI * 180));
        return -2;
    }
    if (!std::isfinite(goal.command.max_effort) || goal.command.max_effort <= 0 || goal.command.max_effort > param.effort_limits) {
        // 非法的最大力矩
        param.result.stalled      = false;
        param.result.reached_goal = false;
        param.new_result          = true;
        printf("%s[%d]: Rejected, bad max_effort %.2f (0, %.2f]\n", name_, id, goal.command.max_effort, param.effort_limits);
        return -3;
    }
    if (emergency_stop()) {
        // 急停状态, 拒绝指令
        param.result.stalled      = false;
        param.result.reached_goal = false;
        param.new_result          = true;
        printf("%s[%d]: Rejected, emergency stop\n", name_, id);
        return -4;
    }
    if (param.state == State::WAIT_MOTOR) {
        // 尚未连接到驱动器
        param.result.stalled      = false;
        param.result.reached_goal = false;
        param.new_result          = true;
        printf("%s[%d]: Rejected, not connected to motor\n", name_, id);
        return -5;
    }

    // 指令正确, 开始执行操作

    // 上锁, 避免目标信息访问冲突
    InterruptLock lock;
    lock.lock();
    param.goal         = goal;
    param.enable_motor = true;
    param.new_result   = false;
    param.state        = State::STARTING;
    lock.unlock();

    printf("%s[%d]: Accepted, position=%.2f, max_effort=%.2f\n", name_, id, float(param.goal.command.position / M_PI * 180), param.goal.command.max_effort);
    return 0;
}

inline int HandGripController::cancel(int id) {
    if (id < 1 || id > OCServoRS485::motor_count_) {
        // 错误的手爪id
        printf("%s: Cancel, bad hand id %d\n", name_, id);
        return -1;
    }
    Parameter& param = param_[id - 1];

    // 禁用电机并取消操作
    param.enable_motor        = false;
    param.result.stalled      = false;
    param.result.reached_goal = false;
    param.new_result          = true;
    param.state               = State::IDLE;

    printf("%s[%d]: Canceled\n", name_, id);
    return 0;
}

inline int HandGripController::get_feedback(int id, control_msgs::GripperCommandFeedback& fb) {
    if (id < 1 || id > OCServoRS485::motor_count_) {
        // 错误的手爪id
        printf("%s: get_feedback, bad hand id %d\n", name_, id);
        return -1;
    }
    Parameter& param = param_[id - 1];

    if (param.new_feedback) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        param.new_feedback = false;
        fb                 = param.feedback;
        return 0;
    } else {
        return -2;
    }
}

inline int HandGripController::get_result(int id, control_msgs::GripperCommandResult& result) {
    if (id < 1 || id > OCServoRS485::motor_count_) {
        // 错误的手爪id
        printf("%s: get_result, bad hand id %d\n", name_, id);
        return -1;
    }
    Parameter& param = param_[id - 1];

    if (param.new_result) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        param.new_result = false;
        result           = param.result;
        return 0;
    } else {
        return -2;
    }
}

//extern HandGripController hand_ctrl;
}