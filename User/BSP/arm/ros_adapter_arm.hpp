#pragma once

#include <functional>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include "action_server.hpp"
#include "ros_adapter_head.hpp"

#include "arm_trajectory_controller.hpp"

namespace hustac {

/* ArmTrajectoryController 到 ROS action server 的适配器 */
class ROSAdapterArm {
public:
    const char *name_;
    ros::NodeHandle &nh_;
    ArmControllerT &controller_;
    ActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;
    // 发布关节状态
    std::unique_ptr<ROSAdapter::MotorStatePublisher> state_publisher_;
    // 发布反馈信息
    SoftTimerMS<100> timer_pub_fb_ = {0};
    
    ROSAdapterArm(const char *name, ros::NodeHandle &nh, ArmControllerT &controller, const char *action_server_name,
                  const char *joint_state_name)
        : name_(name), nh_(nh), controller_(controller), action_server_(
        action_server_name,
        std::bind(&ROSAdapterArm::on_goal, this, std::placeholders::_2),
        std::bind(&ROSAdapterArm::on_cancel, this)), state_publisher_(
        std::make_unique<ROSAdapter::MotorStatePublisher>(joint_state_name, nh, controller_.arm_, joint_state_name)) {
    }
    void init();
    void spin_once();

private:
    void on_goal(const control_msgs::FollowJointTrajectoryGoal &goal);
    void on_cancel();
};

inline void ROSAdapterArm::init() {
    state_publisher_->init();
    action_server_.start(&nh_);
}

inline void ROSAdapterArm::spin_once() {
    state_publisher_->spin_once();
    
    action_server_.update();
    
    // 发布反馈信息
    timer_pub_fb_.is_timeout([this](uint32_t count) {
        control_msgs::FollowJointTrajectoryFeedback fb;
        if (controller_.get_feedback(fb) == 0) {
            // 有新的反馈信息
            action_server_.publishFeedBack(fb);
            return true;
        }
        return false;
    });
    
    // 获取结果信息
    control_msgs::FollowJointTrajectoryResult *result;
    if (controller_.get_result(&result) == 0) {
        // 运行结束
        if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL) {
            // 成功
            action_server_.setSucceeded(*result);
        } else {
            // 失败
            action_server_.setAborted(*result);
        }
    } else {
        // 还在运行
    }
}

inline void ROSAdapterArm::on_goal(const control_msgs::FollowJointTrajectoryGoal &goal) {
    int ret = controller_.set_goal(goal);
    if (ret == 0) {
        action_server_.setAccepted();
    } else {
        control_msgs::FollowJointTrajectoryResult *result;
        if (controller_.get_result(&result) == 0) {
            action_server_.setRejected(*result);
        } else {
            action_server_.setRejected();
        }
    }
}

inline void ROSAdapterArm::on_cancel() {
    controller_.cancel();
    control_msgs::FollowJointTrajectoryResult *result;
    if (controller_.get_result(&result) == 0) {
        action_server_.setCanceled(*result);
    } else {
        action_server_.setCanceled();
    }
}

using ArmAdapterT = ROSAdapterArm;
//extern ROSAdapterArm ros_arm_right;
//extern ROSAdapterArm ros_arm_left;
}
