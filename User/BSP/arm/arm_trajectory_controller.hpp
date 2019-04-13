#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>

#include <action_server.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "high_resolution_clock.h"
#include "trajectory.hpp"
#include "zeda_motor.hpp"

namespace hustac {

template <size_t joint_count, size_t max_trajectory_points>
class ArmTrajectoryController : virtual public EmergencyStoppable {

#define R2D(rad) ((rad) / M_PI * 180)
#define D2R(deg) ((deg)*M_PI / 180)

public:
    static const int joint_count_ = joint_count;

    // 两个轨迹点之间的最大位移
    float max_delta_position_ = 30 * float(M_PI) / 180;
    // 默认容差值, 小于0表示无限制
    JointTolerance<joint_count> default_path_tol_;
    JointTolerance<joint_count> default_goal_tol_;
    // 关节值限制
    //{0, 0}表示环形关节, 角度无限制, 但是在实际控制时会归一到[-pi,pi)
    // std::array<std::pair<float, float>, joint_count> joint_limits;

    const char* name_;                           // 控制器名称
    ZedaDriver<joint_count_>& arm_;               // 受控的机械臂

    // 当前执行的轨迹
    Trajectory<joint_count, max_trajectory_points> current_trajectory_;

    enum class State {
        WAIT_MOTOR, // 等待电机通信成功
        IDLE,       // 等待指令, 点击使能
        STARTING,   // 准备启动
        WORKING,    // 正在跟踪
    };

    /* 以下变量同时在中断和主函数中使用 */
    volatile State state_         = State::WAIT_MOTOR; // 当前工作状态
    volatile bool new_read_       = false;             // 有新的反馈数据
    volatile bool new_result_     = false;             // 有新的结果
    volatile bool enable_motor_   = false;             // 是否使能电机
    TrajectoryPoint<joint_count> desired_point_;       // 当前设定的轨迹点
    TrajectoryPoint<joint_count> actual_point_;        // 当前实际的轨迹点
    TrajectoryPoint<joint_count> error_point_;         // 当前实际的轨迹点

    /* 轨迹跟踪状态 */
    uint64_t time_start_ = 0; // 轨迹跟踪的开始时间, 单片机时间
    size_t next_point_index_; // 当前插值的下一个轨迹点

    control_msgs::FollowJointTrajectoryResult result_;

    // 线性插值算法
    int linear_interpolate(const TrajectoryPoint<joint_count>& begin, const TrajectoryPoint<joint_count>& end, TrajectoryPoint<joint_count>& current) {
        // 检查时间戳
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            return -1;
        }
        float percent = float(current.time_nsec - begin.time_nsec) / (end.time_nsec - begin.time_nsec);
        for (int i = 0; i < joint_count; i++) {
            float delta_pos = end.positions[i] - begin.positions[i];
            if (arm_.motor(i + 1).limit() == std::pair<float, float>(0, 0)) {
                // 环形关节
                delta_pos = std::remainder(delta_pos, 2 * float(M_PI));
            }
            float pos = begin.positions[i] + percent * delta_pos;
            if (arm_.motor(i + 1).limit() == std::pair<float, float>(0, 0)) {
                // 环形关节
                pos = std::remainder(pos, 2 * float(M_PI));
            }
            current.positions[i] = pos;
        }
        return 0;
    }

    ArmTrajectoryController(const char* _name, ZedaDriver<joint_count_>& _arm)
        : name_(_name)
        , arm_(_arm) {
        std::fill_n(default_path_tol_.positions.begin(), joint_count, D2R(10));
        std::fill_n(default_goal_tol_.positions.begin(), joint_count, D2R(0.5));

        arm_.set_read_pos_callback(std::bind(&ArmTrajectoryController::on_read_pos, this));
        arm_.set_sync_write_callback(std::bind(&ArmTrajectoryController::on_sync_write, this));

        //int dummy;
        //switch (dummy) {
        //case sizeof(current_trajectory_):
        //case sizeof(current_trajectory_):
        // break;
        //}
    }

    void on_emergency_stop_changed(bool value) override {
        if (value) {
            arm_.enable_motor(0, 0);
            InterruptLock lock;
            std::unique_lock<InterruptLock> lock_guard(lock);
            enable_motor_            = false;
            desired_point_           = actual_point_;
            desired_point_.time_nsec = 0;
            lock_guard.unlock();
            _finish_with_error(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, "emergency_stop_");
    
            printf("%s: emergency_stop_!!!\n", name_);
        } else {
            InterruptLock lock;
            std::unique_lock<InterruptLock> lock_guard(lock);
            desired_point_           = actual_point_;
            desired_point_.time_nsec = 0;
            lock_guard.unlock();
    
            printf("%s: emergency_stop_ released\n", name_);
        }
    }

    // 设置关节电机目标位姿
    void _set_motor_positions(std::array<float, joint_count>& positions) {
        for (int i = 0; i < joint_count; i++) {
            arm_.motor(i + 1).goal_position(positions[i]);
        }
    }

    // 对比actual与desired, 计算误差error_point_
    void _compute_error(const TrajectoryPoint<joint_count>& desired, const TrajectoryPoint<joint_count>& actual, TrajectoryPoint<joint_count>&) {
        for (int i = 0; i < joint_count; i++) {
            float error = desired.positions[i] - actual.positions[i];
            if (arm_.motor(i + 1).limit() == std::pair<float, float>(0, 0)) {
                // 环形关节
                error = std::remainder(error, 2 * float(M_PI));
            }
            error_point_.positions[i] = error;
        }
        error_point_.time_nsec = std::max(desired.time_nsec, actual.time_nsec);
    }

    // 检查误差是否在容许范围内
    static bool _is_error_in_tolerance(const TrajectoryPoint<joint_count>& error, const JointTolerance<joint_count>& tolerance, bool display = false) {
        for (int i = 0; i < joint_count; i++) {
//            if (i + 1 < joint_count_) {
//                // 1-6号电机, 不检查
//                continue;
//            }
            if (std::abs(error.positions[i]) > tolerance.positions[i]) {
                if (display) {
                    printf("joint %d: abs(error %f) > tol %f\n", i + 1, R2D(std::abs(error.positions[i])), R2D(tolerance.positions[i]));
                }
                return false;
            }
        }
        return true;
    }

    // 以给定错误停止任务执行
    void _finish_with_error(int32_t error_code, const char* error_string) {
        if (state_ == State::STARTING || state_ == State::WORKING) {
            // 记录执行结果
            result_.error_code   = error_code;
            result_.error_string = error_string;
            new_result_          = true;

            // 关节保持当前实际位置
            state_ = State::IDLE;
        }
    }

    // 检查位置与时间容差, 若超出容差则中止执行, 若达到目标则完成执行
    void _check_tolerance() {
        uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
        // 计算容差
        _compute_error(desired_point_, actual_point_, error_point_);
        if (next_point_index_ < current_trajectory_.points_length) {
            // 正在执行
            if (!_is_error_in_tolerance(error_point_, current_trajectory_.path_tol, true)) {
                // 误差超过容许范围!, 结束执行
                desired_point_           = actual_point_;
                desired_point_.time_nsec = 0;
                _finish_with_error(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, "PATH_TOLERANCE_VIOLATED");
                printf("%s[Result]: PATH_TOLERANCE_VIOLATED\n", name_);
            } else {
                // 误差在容许范围内, 继续执行
            }
        } else {
            // 等待结束
            if (_is_error_in_tolerance(error_point_, current_trajectory_.goal_tol)) {
                // 误差在容许范围, 结束执行

                // 记录执行结果
                result_.error_code   = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                result_.error_string = "";
                new_result_          = true;

                // 返回到空闲状态
                state_ = State::IDLE;

                auto& p = error_point_;
                printf("%s[Result]: SUCCESSFUL error=[", name_);
                for (int i = 0; i < joint_count_; i++) {
                    if (i > 0)
                        printf(",");
                    printf("%.2f", R2D(p.positions[i]));
                }
                printf("]\n");
            } else {
                // 误差超过容许范围
                if (now - time_start_ <= current_trajectory_.points[current_trajectory_.points_length - 1].time_nsec + current_trajectory_.goal_time_tolerance) {
                    // 用时未超过容差范围, 继续等待
                } else {
                    // 用时超出, 强制停止
                    _is_error_in_tolerance(error_point_, current_trajectory_.goal_tol, true);

                    _finish_with_error(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED, "GOAL_TOLERANCE_VIOLATED");
                    printf("%s[Result]: %.3f > %.3f GOAL_TOLERANCE_VIOLATED\n", name_, (now - time_start_) / 1e6f, (current_trajectory_.points[current_trajectory_.points_length - 1].time_nsec + current_trajectory_.goal_time_tolerance) / 1e6f);
                }
            }
        }
    }

    uint32_t last_print_time = 0;

    /* 由机械臂驱动调用的回调 */
    // 当需要写入新位置时的回调
    void on_sync_write() {
        uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
        uint64_t duration;
        switch (state_) {
        case State::WAIT_MOTOR:
            // 电机尚未初始化完毕, 禁用电机
            arm_.enable_motor(0, 0);
            break;
        case State::IDLE:
            if (enable_motor_) {
                _set_motor_positions(desired_point_.positions);
                arm_.enable_motor(1, 7);
                // 更新误差
                _compute_error(desired_point_, actual_point_, error_point_);
            } else {
                arm_.enable_motor(0, 0);
            }
            break;
        case State::STARTING:
            // 启动操作, 切换到执行状
            enable_motor_ = true;
            arm_.enable_motor(1, 7);

            time_start_       = now;
            next_point_index_ = 0;
            state_            = State::WORKING;
            printf("%s: STARTING -> WORKING\n", name_);
            // don't break here
        case State::WORKING:
            // 继续执行操作

            // 计算距离任务开始的时间
            duration = now - time_start_;
            // 找到下一个要跟踪的轨迹点
            while (duration > current_trajectory_.points[next_point_index_].time_nsec && next_point_index_ < current_trajectory_.points_length) {
                next_point_index_++;
            }

            // 计算目标位置
            if (next_point_index_ == 0) {
                // 起始
                desired_point_ = current_trajectory_.points[0];
                // 设定电机状态
                _set_motor_positions(desired_point_.positions);
            } else if (next_point_index_ < current_trajectory_.points_length) {
                // 中间状态, 计算插值点
                desired_point_.time_nsec = duration;
                // 线性插值
                if (linear_interpolate(current_trajectory_.points[next_point_index_ - 1], current_trajectory_.points[next_point_index_], desired_point_) != 0) {
                    _Error_Handler((char*)__FILE__, __LINE__);
                }
                // 设定电机状态
                _set_motor_positions(desired_point_.positions);
            } else {
                // 正在结束
                desired_point_ = current_trajectory_.points[current_trajectory_.points_length - 1];
                _set_motor_positions(desired_point_.positions);
            }

            // 打印状态
            if (last_print_time == 0 || HAL_GetTick() - last_print_time > 100) {
                last_print_time = HAL_GetTick();
                auto& p         = desired_point_;
                printf("%s[WORKING]: %d/%d %.3f sec [", name_, next_point_index_, current_trajectory_.points_length, (now - time_start_) / 1e9f);
                for (int i = 0; i < joint_count_; i++) {
                    if (i > 0)
                        printf(",");
                    printf("%.2f", R2D(p.positions[i]));
                }
                printf("]\n");
            }

            // 检查容差
            _check_tolerance();
        }
    }

    // 当读取到新位置时的回调
    void on_read_pos() {
        if (state_ == State::WAIT_MOTOR) {
            // 首次读取到电机反馈数据后, 认为电机初始化完毕
            // 电机初始化完毕后, 默认不启动电机
            enable_motor_ = false;
            state_        = State::IDLE;
            printf("%s[IDLE]: Connected to motor\n", name_);
        }

        // 将电机位置反馈记录到actual_point_
        actual_point_.time_nsec = MY_GetNanoSecFromCycle(MY_GetCycleCount());
        actual_point_.positions = arm_.position_buffer();
        new_read_               = true;

        if (state_ == State::WORKING) {
            _check_tolerance();
        }
    }

    // 由主循环调用, 设定目标
    // 若尚未与电机建立通信, 则返回-1
    // 若轨迹结构错误, 则取消操作并停在原位, 返回-2
    // 若轨迹值非法, 则禁用电机, 返回-3
    int set_goal(const control_msgs::FollowJointTrajectoryGoal& msg) {
        if (emergency_stop()) {
            // 急停状态
            printf("%s: Goal rejected, emergency_stop_\n", name_);
            result_.error_code   = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            result_.error_string = "emergency_stop_";
            new_result_          = true;
            return -4;
        }
        if (state_ == State::WAIT_MOTOR) {
            // 尚未与电机通信成功
            printf("%s: Goal rejected, WAIT_MOTOR\n", name_);
            result_.error_code   = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            result_.error_string = "wait motor";
            new_result_          = true;
            return -1;
        }

        // 检查轨迹消息
        auto parser = TrajectoryParser<joint_count, max_trajectory_points>(arm_.joint_names(), msg, current_trajectory_);

        int ret = parser.check_trajectory_struct();
        if (ret <= 0) {
            // 轨迹结构错误, 关节保持原位

            InterruptLock lock;
            lock.lock();
            enable_motor_            = true;
            desired_point_           = actual_point_;
            desired_point_.time_nsec = 0;
            state_                   = State::IDLE;
            lock.unlock();

            auto& p = desired_point_;
            printf("%s: Goal rejected, bad msg (%d), hold pos [", name_, ret);
            for (int i = 0; i < joint_count_; i++) {
                if (i > 0)
                    printf(",");
                printf("%.2f", R2D(p.positions[i]));
            }
            printf("]\n");

            result_.error_code   = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            result_.error_string = "INVALID_JOINTS, hold pos";
            new_result_          = true;
            return -2;
        } else {
            if (parser.check_trajectory_value(arm_.joint_limits(), max_delta_position_) < 0) {
                // 轨迹数据错误, 需要禁用电机
    
                InterruptLock lock;
                lock.lock();
                enable_motor_ = false;
                state_        = State::IDLE;
                lock.unlock();

                printf("%s: Goal rejected, bad traj value(%d), disable motor\n", name_, ret);
                result_.error_code   = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
                result_.error_string = "INVALID_GOAL, disable motor";
                new_result_          = true;
                return -3;
            } else {
                // 轨迹正确, 开始执行动作
    
                InterruptLock lock;
                lock.lock();
                enable_motor_ = true;
                new_result_   = false;
                // 首先置为空闲状态, 避免points访问冲突
                state_ = State::IDLE;
                lock.unlock();

                // 拷贝容差值
                parser.copy_tolerance(default_path_tol_, default_goal_tol_);
                // 拷贝轨迹数据
                parser.copy_trajectory(arm_.joint_limits());

                return _execute_goal();
            }
        }
    }

    // 直接执行保存在flash中的轨迹
    int set_goal(int page) {
        if (emergency_stop()) {
            // 急停状态
            printf("%s: Goal rejected, emergency_stop_\n", name_);
            result_.error_code   = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            result_.error_string = "emergency_stop_";
            new_result_          = true;
            return -4;
        }
        if (state_ == State::WAIT_MOTOR) {
            // 尚未与电机通信成功
            printf("%s: Goal rejected, WAIT_MOTOR\n", name_);
            result_.error_code   = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            result_.error_string = "wait motor";
            new_result_          = true;
            return -1;
        }

        int ret = current_trajectory_.load(page);
        if (ret == 0) {
            if (current_trajectory_.points_length <= 0) {
                return -1;
            }
            return _execute_goal();
        }
        return ret;
    }

    int _execute_goal() {
        printf("%s: Goal accepted, STARTING\n", name_);

        current_trajectory_.print();

        // 置为等待启动状态
        state_ = State::STARTING;
        return 0;
    }

    // 由主循环调用, 取消当前操作
    int cancel() {
        // 设定空消息, 即为取消操作
        printf("%s: Canceled\n", name_);
        set_goal(control_msgs::FollowJointTrajectoryGoal());
        result_.error_code   = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
        result_.error_string = "canceled";
        new_result_          = true;
        return 0;
    }

    // 由主循环调用, 获取当前反馈信息, 当没有新的反馈信息或未启动时, 返回-1
    int get_feedback(control_msgs::FollowJointTrajectoryFeedback& fb) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        
        int ret;
        if (new_read_) {

            fb.header.seq = arm_.count_read_;
            fb.header.stamp.fromNSec(arm_.last_recv_ok_);
            fb.header.frame_id          = "";
            fb.joint_names_length       = arm_.joint_names().size();
            fb.joint_names              = (char**)arm_.joint_names().data();
            fb.actual.positions_length  = (uint32_t)joint_count_;
            fb.actual.positions         = actual_point_.positions;
            fb.desired.positions_length = (uint32_t)joint_count_;
            fb.desired.positions        = desired_point_.positions;
            fb.error.positions_length   = (uint32_t)joint_count_;
            fb.error.positions          = error_point_.positions;

            new_read_ = false;
            ret      = 0;
        } else {
            ret = -1;
        }
        return ret;
    }

    // 由主循环调用, 获取结果
    int get_result(control_msgs::FollowJointTrajectoryResult** result_out) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        
        int ret;
        if (new_result_) {
            *result_out = &result_;
            new_result_  = false;
            ret         = 0;
        } else {
            *result_out = NULL;
            ret         = -1;
        }

        return ret;
    }

#undef R2D
#undef D2R
};

using ArmControllerT = ArmTrajectoryController<ArmMotorT::motor_count_, 200>;

extern uint8_t ARM_RIGHT_CTRL_MEM_REGION[sizeof(ArmControllerT)];
extern uint8_t ARM_LEFT_CTRL_MEM_REGION[sizeof(ArmControllerT)];
}