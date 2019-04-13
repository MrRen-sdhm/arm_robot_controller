#pragma once

#include <functional>
#include <memory>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <rosserial/control_msgs/SingleJointPositionAction.h>
#include <rosserial/std_msgs/Float64.h>
#include "action_server.hpp"

#include "ocservo_ttl.hpp"
#include "pwm_servo.hpp"

namespace hustac {

namespace ROSAdapter {

class MotorStatePublisher {
public:
    const char* name_;
    ros::NodeHandle& nh_;
    AbstractMotorDriver& driver_;
    // 发布关节状态
    sensor_msgs::JointState msg_joint_;
    ros::Publisher pub_joint_;
    uint32_t count_pub_joint_ = 0; // 成功发送关节状态的次数
    uint32_t last_count_read_ = 0; // 上一次发送关节状态的序号
    SoftTimerNS<> timer_pub_ = {0};
    
    MotorStatePublisher(const char* name, ros::NodeHandle& nh, AbstractMotorDriver& driver,
                        const char* joint_state_name, uint64_t min_interval_us = 0)
        : name_(name), nh_(nh), driver_(driver), pub_joint_(joint_state_name, &msg_joint_) {
        msg_joint_.name_length = (uint32_t) driver_.motor_size();
        msg_joint_.name = new char* [driver_.motor_size()];
        msg_joint_.position_length = (uint32_t) driver_.motor_size();
        msg_joint_.position = new float[driver_.motor_size()];
        msg_joint_.velocity_length = (uint32_t) driver_.motor_size();
        msg_joint_.velocity = new float[driver_.motor_size()];
        msg_joint_.effort_length = (uint32_t) driver_.motor_size();
        msg_joint_.effort = new float[driver_.motor_size()];
        timer_pub_.set_interval(min_interval_us);
    }
    
    ~MotorStatePublisher() {
        delete[] msg_joint_.name;
        delete[] msg_joint_.position;
        delete[] msg_joint_.velocity;
        delete[] msg_joint_.effort;
    }
    
    void init() {
        timer_pub_.reset();
        nh_.advertise(pub_joint_);
        for (int id = 1; id <= driver_.motor_size(); id++) {
            auto& motor = driver_.motor(id);
            motor.init();
            msg_joint_.name[id - 1] = (char*) motor.name();
        }
    }
    
    void spin_once() {
        // 发布关节状态
        timer_pub_.is_timeout([this](uint32_t count) -> bool {
            if (driver_.count_read_ > last_count_read_) {
                
                msg_joint_.header.stamp = nh_.now();
                do {
                    InterruptLock lock;
                    InterruptLockGuard lock_guard(lock);
                    last_count_read_ = driver_.count_read_;
                    msg_joint_.header.seq = driver_.count_read_;
                    for (int id = 1; id <= driver_.motor_size(); id++) {
                        auto& motor = driver_.motor(id);
                        msg_joint_.position[id - 1] = motor.current_position();
                        msg_joint_.velocity[id - 1] = motor.current_velocity();
                        msg_joint_.effort[id - 1] = motor.current_effort();
                    }
                } while (0);
                
                if (pub_joint_.publish(&msg_joint_) < 0) {
                    printf("node_handler: failed to publish hand_state\n");
                } else {
                    count_pub_joint_++;
                    return true;
                }
            }
            return false;
        });
    }
};

class SingleMotorPositionSubscriber : virtual public EmergencyStoppable {
public:
    const char* name_;
    ros::NodeHandle& nh_;
    BaseMotor& motor_;
    ros::Subscriber<std_msgs::Float64> sub_pos_;
    
    SingleMotorPositionSubscriber(const char* name, ros::NodeHandle& nh_, BaseMotor& motor, const char* sub_topic_)
        : name_(name), nh_(nh_), motor_(motor),
          sub_pos_(sub_topic_, std::bind(&SingleMotorPositionSubscriber::on_pos, this, std::placeholders::_1)) {}
    void init() {
        motor_.init();
        nh_.subscribe(sub_pos_);
    }
    void spin_once() {
        motor_.spin_once();
    }
    void on_emergency_stop_changed(bool value) override {
        if (value) {
            motor_.stop();
        }
    }
private:
    void on_pos(const std_msgs::Float64& msg) {
        if (emergency_stop()) {
            return;
        }
        if (!std::isfinite(msg.data)) {
            motor_.stop();
            return;
        }
        motor_.goal_position(msg.data);
        motor_.start();
    }
};

class SingleMotorPositionAction : virtual public EmergencyStoppable {
public:
    const char* name_;
    ros::NodeHandle& nh_;
    BaseMotor& motor_;
    ActionServer<control_msgs::SingleJointPositionAction> action_server_;
    bool working_ = false;
    SoftTimerMS<0, false> timer_pub_fb_ = {0};
    uint32_t count_fb_ = 0;
    float goal_position_ = std::numeric_limits<float>::signaling_NaN();
    float tolerance_;
    
    SingleMotorPositionAction(const char* name, ros::NodeHandle& nh_, BaseMotor& motor, const char* action_server_name,
                              float tolerance, uint32_t pub_fb_interval)
        : name_(name), nh_(nh_), motor_(motor), action_server_(
        action_server_name,
        std::bind(&SingleMotorPositionAction::on_goal, this, std::placeholders::_2),
        std::bind(&SingleMotorPositionAction::on_cancel, this)), tolerance_(tolerance) {
        timer_pub_fb_.set_interval(pub_fb_interval);
    }
    void init();
    void spin_once();
    void on_emergency_stop_changed(bool value) override;
private:
    void on_goal(const control_msgs::SingleJointPositionGoal& goal);
    void on_cancel();
};

inline void SingleMotorPositionAction::init() {
    motor_.init();
    action_server_.start(&nh_);
}

inline void SingleMotorPositionAction::spin_once() {
    action_server_.update();
    
    if (working_) {
        float pos, vel, eff;
        do {
            InterruptLock lock;
            InterruptLockGuard lock_guard(lock);
            pos = motor_.current_position();
            vel = motor_.current_velocity();
        } while (0);
        float error = pos - goal_position_;
        
        // 发布反馈信息
        bool published_fb = false;
        if (timer_pub_fb_.is_timeout()) {
            control_msgs::SingleJointPositionFeedback fb;
            fb.header.stamp = nh_.now();
            fb.header.seq = count_fb_++;
            fb.position = pos;
            fb.velocity = vel;
            fb.error = error;
            action_server_.publishFeedBack(fb);
            published_fb = true;
        }
        
        // 发布结果信息
        if (std::abs(error) <= tolerance_) {
            // 运行结束
            if (!published_fb) {
                control_msgs::SingleJointPositionFeedback fb;
                fb.header.stamp = nh_.now();
                fb.header.seq = count_fb_++;
                fb.position = pos;
                fb.velocity = vel;
                fb.error = error;
                action_server_.publishFeedBack(fb);
            }
            working_ = false;
            action_server_.setSucceeded();
            printf("%s: Motor %s action finished! goal=%.3f current=%.3f error=%.3f\n", name_, motor_.name(),
                   goal_position_, pos, error);
        } else {
            // 还在运行
        }
    }
    motor_.spin_once();
}

inline void SingleMotorPositionAction::on_emergency_stop_changed(bool value) {
    if (value) {
        if (working_) {
            motor_.stop();
            working_ = false;
            action_server_.setAborted(control_msgs::SingleJointPositionResult(), "emergency stop");
            printf("%s: Motor %s action finished! emergency stop\n", name_, motor_.name());
        }
    } else {
    
    }
}

inline void SingleMotorPositionAction::on_goal(const control_msgs::SingleJointPositionGoal& goal) {
    if (emergency_stop()) {
        working_ = false;
        action_server_.setRejected(control_msgs::SingleJointPositionResult(), "emergency stop");
        printf("%s: Motor %s goal_pos = %.3f. Rejected! Emergency stop\n", name_, motor_.name(), goal_position_);
        return;
    }
    goal_position_ = goal.position;
    int ret = motor_.goal_position(goal_position_);
    if (ret < 0) {
        working_ = false;
        action_server_.setRejected(control_msgs::SingleJointPositionResult(), "cannot set goal");
        printf("%s: Motor %s goal_pos = %.3f failed\n", name_, motor_.name(), goal_position_);
        return;
    }
    ret = motor_.start();
    if (ret < 0) {
        working_ = false;
        action_server_.setRejected(control_msgs::SingleJointPositionResult(), "cannot start");
        printf("%s: Motor %s start() failed\n", name_, motor_.name());
        return;
    }
    working_ = true;
    count_fb_ = 0;
    action_server_.setAccepted();
    goal_position_ = motor_.goal_position();
    printf("%s: Motor %s goal_pos = %.3f\n", name_, motor_.name(), goal_position_);
}

inline void SingleMotorPositionAction::on_cancel() {
    motor_.stop();
    working_ = false;
    action_server_.setCanceled();
    printf("%s: Motor %s action cancelled\n", name_, motor_.name());
}

class PointHeadAction : virtual public EmergencyStoppable {
//public:
//    const char *name_;
//    ros::NodeHandle &nh_cpp_;
//    BaseMotor &motor_yaw_;
//    BaseMotor &motor_pitch_;
//    ActionServer<control_msgs::SingleJointPositionAction> action_server_;
//    bool working_ = false;
//    SoftTimerMS<0, false> timer_pub_fb_ = {0};
//    uint32_t count_fb_ = 0;
//    float goal_position_ = std::numeric_limits<float>::signaling_NaN();
//    float tolerance_;
//
//    SingleMotorPositionAction(const char *name, ros::NodeHandle &nh_cpp_, BaseMotor &motor, const char *action_server_name,
//                        float tolerance, uint32_t pub_fb_interval)
//        : name_(name), nh_cpp_(nh_cpp_), motor_(motor), action_server_(
//        action_server_name,
//        std::bind(&SingleMotorPositionAction::on_goal, this, std::placeholders::_2),
//        std::bind(&SingleMotorPositionAction::on_cancel, this)), tolerance_(tolerance) {
//        timer_pub_fb_.set_interval(pub_fb_interval);
//    }
//    void init();
//    void spin_once();
//    void on_emergency_stop_changed(bool value) override;
//private:
//    void on_goal(const control_msgs::SingleJointPositionGoal &goal);
//    void on_cancel();
};

class HeadIndividual {
public:
    const char* name_;
    std::unique_ptr<char[]> name_state_pub_;
    OCServoTTL& driver_;
    static constexpr std::array<float, OCServoTTL::motor_count_> start_pos_ = {0};
    bool reach_start_pos_ = false;
    std::unique_ptr<MotorStatePublisher> state_publisher_;
    std::array<std::unique_ptr<RMLController>, OCServoTTL::motor_count_> rml_ctrls_;
    std::array<std::unique_ptr<char[]>, OCServoTTL::motor_count_> position_ctrl_names_;
    std::array<std::unique_ptr<char[]>, OCServoTTL::motor_count_> position_ctrl_action_names_;
    std::array<std::unique_ptr<SingleMotorPositionAction>, OCServoTTL::motor_count_> position_actions_;
    std::array<std::unique_ptr<char[]>, OCServoTTL::motor_count_> position_sub_names_;
    std::array<std::unique_ptr<char[]>, OCServoTTL::motor_count_> position_topic_names_;
    std::array<std::unique_ptr<SingleMotorPositionSubscriber>, OCServoTTL::motor_count_> position_topics_;
    
    HeadIndividual(const char* name, ros::NodeHandle& nh, OCServoTTL& driver,
                   const char* joint_state_name, const char* action_server_name, const char* pos_sub_name
    ) :
        name_(name),
        driver_(driver) {
        char tmp[64];
        
        snprintf(tmp, sizeof(tmp), "%s_STATE_PUB", name_);
        name_state_pub_ = std::make_unique<char[]>(strlen(tmp) + 1);
        strcpy(name_state_pub_.get(), tmp);
        
        state_publisher_ = std::make_unique<MotorStatePublisher>(name_state_pub_.get(), nh, driver_, joint_state_name,
                                                                 10000000);
        
        for (int i = 0; i < OCServoTTL::motor_count_; i++) {
            rml_ctrls_[i] = std::make_unique<RMLController>(driver_.motor(i + 1).name(), driver_.motor(i + 1), 10,
                                                            120 / 180.0f * M_PI, 180 / 180.0f * M_PI);
            
            snprintf(tmp, sizeof(tmp), "%s_POS_CTRL_%d", name_, i + 1);
            position_ctrl_names_[i] = std::make_unique<char[]>(strlen(tmp) + 1);
            strcpy(position_ctrl_names_[i].get(), tmp);
            
            snprintf(tmp, sizeof(tmp), "%s/%d", action_server_name, i + 1);
            position_ctrl_action_names_[i] = std::make_unique<char[]>(strlen(tmp) + 1);
            strcpy(position_ctrl_action_names_[i].get(), tmp);
            
            position_actions_[i] = std::make_unique<SingleMotorPositionAction>(position_ctrl_names_[i].get(), nh,
                                                                               *rml_ctrls_[i],
                                                                               position_ctrl_action_names_[i].get(),
                                                                               1.0f / 180.0f * float(M_PI), 100);
            
            snprintf(tmp, sizeof(tmp), "%s_POS_SUB_%d", name_, i + 1);
            position_sub_names_[i] = std::make_unique<char[]>(strlen(tmp) + 1);
            strcpy(position_sub_names_[i].get(), tmp);
            
            snprintf(tmp, sizeof(tmp), "%s/%d", pos_sub_name, i + 1);
            position_topic_names_[i] = std::make_unique<char[]>(strlen(tmp) + 1);
            strcpy(position_topic_names_[i].get(), tmp);
            
            position_topics_[i] = std::make_unique<SingleMotorPositionSubscriber>(position_sub_names_[i].get(), nh,
                                                                                  *rml_ctrls_[i],
                                                                                  position_topic_names_[i].get());
        }
    }
    
    void init() {
        state_publisher_->init();
        for (int i = 0; i < OCServoTTL::motor_count_; i++) {
            position_actions_[i]->init();
            position_topics_[i]->init();
        }
    }
    
    void spin_once() {
        if (!reach_start_pos_ && driver_.count_read_ > 0) {
            // 上电回到指定初始位置
            bool failed = false;
            for (int i = 0; i < OCServoTTL::motor_count_; i++) {
                if (rml_ctrls_[i]->goal_position(start_pos_[i]) < 0) {
                    failed = true;
                    break;
                }
                if (rml_ctrls_[i]->start() < 0) {
                    failed = true;
                    break;
                }
            }
            if (!failed) {
                printf("%s: Goto start pos\n", name_);
            } else {
                printf("%s: Failed to goto start pos\n", name_);
            }
            reach_start_pos_ = true;
        }
        state_publisher_->spin_once();
        for (int i = 0; i < OCServoTTL::motor_count_; i++) {
            position_actions_[i]->spin_once();
            position_topics_[i]->spin_once();
        }
    }
};

class HeadUniversal {

};

}

}
