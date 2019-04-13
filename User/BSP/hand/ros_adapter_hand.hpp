#pragma once

#include <functional>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include "action_server.hpp"

#include "hand_grip_controller.hpp"

namespace hustac {

/* HandGripController 到 ROS action server 的适配器 */
class ROSAdapterHandSingle {
public:
    const char* name_;
    ros::NodeHandle& nh_;
    HandGripController& controller_;
    uint8_t id_;
    ActionServer<control_msgs::GripperCommandAction> action_server_;

    ROSAdapterHandSingle(const char* name, ros::NodeHandle& nh_, HandGripController& controller, uint8_t id, const char* action_server_name)
        : name_(name)
        , nh_(nh_)
        , controller_(controller)
        , id_(id)
        , action_server_(
              action_server_name,
              std::bind(&ROSAdapterHandSingle::on_goal, this, std::placeholders::_2),
              std::bind(&ROSAdapterHandSingle::on_cancel, this)) {
    }
    void init();
    void spin_once();

private:
    void on_goal(const control_msgs::GripperCommandGoal& goal);
    void on_cancel();
};

inline void ROSAdapterHandSingle::init() {
    action_server_.start(&nh_);
}

inline void ROSAdapterHandSingle::spin_once() {
    action_server_.update();

    // 发布关节状态

    // 发布反馈信息
    control_msgs::GripperCommandFeedback fb;
    if (controller_.get_feedback(id_, fb) == 0) {
        // 有新的反馈信息
        action_server_.publishFeedBack(fb);
    }

    // 获取结果信息
    control_msgs::GripperCommandResult result;
    if (controller_.get_result(id_, result) == 0) {
        // 运行结束
        action_server_.setSucceeded(result);
    } else {
        // 还在运行
    }
}

inline void ROSAdapterHandSingle::on_goal(const control_msgs::GripperCommandGoal& goal) {
    int ret = controller_.set_goal(id_, goal);
    if (ret == 0) {
        action_server_.setAccepted();
    } else {
        control_msgs::GripperCommandResult result;
        if (controller_.get_result(id_, result) == 0) {
            action_server_.setRejected(result);
        } else {
            action_server_.setRejected();
        }
    }
}

inline void ROSAdapterHandSingle::on_cancel() {
    controller_.cancel(id_);
    control_msgs::GripperCommandResult result;
    if (controller_.get_result(id_, result) == 0) {
        action_server_.setCanceled(result);
    } else {
        action_server_.setCanceled();
    }
}

class ROSAdapterHand {
public:
    const char* name_;
    ros::NodeHandle& nh_;
    HandGripController& controller_;
    std::array<ROSAdapterHandSingle, OCServoRS485::motor_count_> adapters_;
    // 发布关节状态
    sensor_msgs::JointState msg_joint_;
    ros::Publisher pub_joint_;
    uint32_t count_pub_joint_  = 0; // 成功发送关节状态的次数
    uint32_t last_count_read_ = 0; // 上一次发送关节状态的序号

    ROSAdapterHand(const char* name, ros::NodeHandle& nh_, HandGripController& controller, const char* joint_state_name)
        : name_(name)
        , nh_(nh_)
        , controller_(controller)
        , adapters_{
            ROSAdapterHandSingle("ROS-HAND-R", nh_, controller_, 1, "hand/right"), ROSAdapterHandSingle("ROS-HAND-L", nh_, controller_, 2, "hand/left")
        }
        , pub_joint_(joint_state_name, &msg_joint_) {
    }
        
    void init() {
        nh_.advertise(pub_joint_);
        for (auto& adapter : adapters_) {
            adapter.init();
        }
    }
    
    void spin_once() {
        for (auto& adapter : adapters_) {
            adapter.spin_once();
        }
        // 发布关节状态
        if (controller_.hand_.count_read_ > last_count_read_) {
    
            msg_joint_.header.stamp    = nh_.now();
            msg_joint_.name_length     = controller_.hand_.motor_count_;
            msg_joint_.name            = (char**)controller_.hand_.joint_names().data();
            msg_joint_.position_length = controller_.hand_.motor_count_;
            msg_joint_.velocity_length = controller_.hand_.motor_count_;
            msg_joint_.effort_length   = controller_.hand_.motor_count_;
            do {
                InterruptLock lock;
                InterruptLockGuard lock_guard(lock);
                last_count_read_ = controller_.hand_.count_read_;
                msg_joint_.header.seq = controller_.hand_.count_read_;
                msg_joint_.position        = (float*)controller_.hand_.position_buffer().data();
                msg_joint_.velocity        = (float*)controller_.hand_.velocity_buffer().data();
                msg_joint_.effort          = (float*)controller_.hand_.effort_buffer().data();
            } while (0);

            if (pub_joint_.publish(&msg_joint_) < 0) {
                printf("node_handler: failed to publish hand_state\n");
            } else {
                count_pub_joint_++;
            }
        }
    }
};

}
