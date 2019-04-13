#pragma once

#include <array>

#include "dmabuffer_uart.hpp"
#include "arm_trajectory_controller.hpp"

namespace hustac {
    
template <typename ControllerT>
class ArmDemo {
public:
    ArmDemo(ControllerT& controller)
        : controller_(controller){
    }
        
    void start();
    
    void stop();

    void spin_once();
    
private:
    ControllerT& controller_;
    std::array<int, 1> step_start_ = {0};
    std::array<int, 6> step_loop_ = {1, 2, 3, 4, 5, 6};
    std::array<int, 1> step_stop_ = {7};
    
    enum class Status {
        IDLE,       // 停止在零位
        STARTING,   // 正在开始
        LOOP,       // 正在循环执行
        STOPING,    // 正在停止
    };
    
    Status status_ = Status::IDLE;
    int step_index_ = 0;
    bool request_run_ = false;
};

template <typename ControllerT>
void ArmDemo<ControllerT>::start() {
    request_run_ = true;
    printf("ArmDemo: request to start\n");
}

template <typename ControllerT>
void ArmDemo<ControllerT>::stop() {
    request_run_ = false;
    printf("ArmDemo: request to stop\n");
}

template <typename ControllerT>
void ArmDemo<ControllerT>::spin_once() {
    if (status_ == Status::IDLE) {
        if (!request_run_) {
//            for (int i = 0; i < ControllerT::joint_count_; i++) {
//                controller_.arm.motor[i].goal_position(0);
//            }
        } else {
            // 开始执行
            status_ = Status::STARTING;
            step_index_ = 0;
            printf("ArmDemo: Starting...\n");
        }
    } else if (status_ == Status::STARTING) {
        if (controller_.state_ == ControllerT::State::IDLE) {
            // 轨迹执行完毕
            if (step_index_ < step_start_.size()) {
                // 执行初始化轨迹
                controller_.set_goal(step_start_[step_index_]);
                printf("ArmDemo: Execute start step %d\n", step_index_);
                step_index_++;
            } else {
                // 初始化轨迹执行完毕
                status_ = Status::LOOP;
                step_index_ = 0;
                printf("ArmDemo: Loop...\n");
            }
        } else {
            // 轨迹正在执行
        }
    } else if (status_ == Status::LOOP) {
        if (controller_.state_ == ControllerT::State::IDLE) {
            // 执行循环轨迹
            if (!request_run_ && step_index_ == 0) {
                status_ = Status::STOPING;
                step_index_ = 0;
                printf("ArmDemo: Stoping...\n");
            } else {
                controller_.set_goal(step_loop_[step_index_]);
                printf("ArmDemo: Execute loop step %d\n", step_index_);
                step_index_ = (step_index_ + 1) % step_loop_.size();
            }
        } else {
            // 轨迹正在执行
        }
    } else if (status_ == Status::STOPING) {
        if (controller_.state_ == ControllerT::State::IDLE) {
            // 轨迹执行完毕
            if (step_index_ < step_stop_.size()) {
                // 执行结束轨迹
                controller_.set_goal(step_stop_[step_index_]);
                printf("ArmDemo: Execute stop step %d\n", step_index_);
                step_index_++;
            } else {
                // 结束轨迹执行完毕
                status_ = Status::IDLE;
                step_index_ = 0;
                printf("ArmDemo: IDLE\n");
            }
        } else {
            // 轨迹正在执行
        }
    }
    
}
    
}