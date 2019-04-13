#pragma once

#include <functional>

#include <ros.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "dmabuffer_uart.hpp"

namespace hustac {

// Action Server for rosserial
// Author: imtoumao@gmail.com
// Difference with startdard ActionServer:
// 1. only one goal is activated, new goal will cancel previous goal immediately.
// 2. status won't be published once goal finished
// 3. goal should be handled immediately in goal_callback (setRejected or setAccepted). so actually there is no PENDING state, goal won't be RECALLED
// 4. cancel should be handled imemdiately in calcel_callback (setCancelled, setAborted  or setSucceeded)
// 5. all function should be called in main thread
// 6. actual posiible status: ACTIVE, SUCCEEDED, ABORTED, PREEMPTED, REJECTED

template<class ActionSpec>
class ActionServer {
public:
    typedef typename ActionSpec::_action_goal_type ActionGoal;
    typedef typename ActionGoal::_goal_type Goal;
    typedef typename ActionSpec::_action_result_type ActionResult;
    typedef typename ActionResult::_result_type Result;
    typedef typename ActionSpec::_action_feedback_type ActionFeedback;
    typedef typename ActionFeedback::_feedback_type Feedback;
private:
    const char *name_;
    std::function<void(ActionServer<ActionSpec> &, const Goal &)> goal_callback_;
    std::function<void(ActionServer<ActionSpec> &)> cancel_callback_;

    char *sub_goal_name_ = NULL;
    ros::Subscriber<ActionGoal> sub_goal_;
    char *sub_cancel_name_ = NULL;
    ros::Subscriber<actionlib_msgs::GoalID> sub_cancel_;
    char *pub_status_name_ = NULL;
    actionlib_msgs::GoalStatusArray pub_status_msg_;
    ros::Publisher pub_status_;
    char *pub_feedback_name_ = NULL;
    ActionFeedback pub_feedback_msg_;
    ros::Publisher pub_feedback_;
    char *pub_result_name_ = NULL;
    ActionResult pub_result_msg_;
    ros::Publisher pub_result_;

    ros::NodeHandle *nh_ = NULL;

    uint32_t count_pub_status_ = 0;
    uint32_t count_pub_feedback_ = 0;
    uint32_t count_pub_result_ = 0;

    static const uint8_t pub_status_interval_ = 100; // ms
    uint32_t last_pub_status_ = 0;                   // ms

    static const size_t goal_status_text_max_length_ = 128;
    char goal_status_text_[goal_status_text_max_length_] = "";

    static const size_t goal_id_text_max_length_ = 128;
    char goal_id_text_[goal_id_text_max_length_] = "";
    actionlib_msgs::GoalStatus goal_status_;

public:

    ActionServer(const char *name,
                 decltype(goal_callback_) goal_cb = decltype(goal_callback_)(),
                 decltype(cancel_callback_) cancel_cb = decltype(cancel_callback_)());

    ~ActionServer() {
        if (sub_goal_name_) free(sub_goal_name_);
        if (sub_cancel_name_) free(sub_cancel_name_);
        if (pub_status_name_) free(pub_status_name_);
        if (pub_feedback_name_) free(pub_feedback_name_);
        if (pub_result_name_) free(pub_result_name_);
    }

    void start(ros::NodeHandle *_nh);

    void registerGoalCallback(std::function<void(const Goal &)> goal_cb);

    void registerCancelCallback(std::function<void()> cancel_cb);

    const actionlib_msgs::GoalID &getGoalID();

    const actionlib_msgs::GoalStatus &getGoalStatus();

    void setAborted(const Result &result = Result(), const char *text = "");

    void setAccepted(const char *text = "");

    void setCanceled(const Result &result = Result(), const char *text = "");

    void setRejected(const Result &result = Result(), const char *text = "");

    void setSucceeded(const Result &result = Result(), const char *text = "");

    void publishFeedBack(const Feedback &feedback);

    void update();

private:
    void _on_goal(const ActionGoal &_action_goal);

    void _on_cancel(const actionlib_msgs::GoalID &_goal_id);

    void _pub_status_array(const actionlib_msgs::GoalStatus &goal_status) {
//        printf("ActionServer %s: %s = %s\n", name, goal_status.goal_id.id, strstatus(goal_status.status));

        pub_status_msg_.header.seq = count_pub_status_++;
        pub_status_msg_.header.stamp = nh_->now();
        pub_status_msg_.header.frame_id = "";
        pub_status_msg_.status_list_length = 1;
        pub_status_msg_.status_list = (actionlib_msgs::GoalStatus *) &goal_status;

        last_pub_status_ = HAL_GetTick();
        pub_status_.publish(&pub_status_msg_);
    }

    void _pub_status_array_none() {
        pub_status_msg_.header.seq = count_pub_status_++;
        pub_status_msg_.header.stamp = nh_->now();
        pub_status_msg_.header.frame_id = "";
        pub_status_msg_.status_list_length = 0;
        pub_status_msg_.status_list = NULL;

        last_pub_status_ = HAL_GetTick();
        pub_status_.publish(&pub_status_msg_);
    }

    void _pub_status(const actionlib_msgs::GoalID &goal_id, uint8_t status, const char *text = "") {
//        printf("ActionServer %s: Goal %s, change to %s\n", name, goal_id.id, strstatus(status));

        actionlib_msgs::GoalStatus goal_status;
        goal_status.goal_id = goal_id;
        goal_status.status = status;
        goal_status.text = text;

        _pub_status_array(goal_status);
    }

    void _change_pub_status(uint8_t status, const char *text = "") {
//        printf("ActionServer %s: %s %s -> %s\n", name, goal_status.goal_id.id, strstatus(goal_status.status), strstatus(status));
        goal_status_.status = status;
        goal_status_.text = text;

        _pub_status_array(goal_status_);
    }

    void _change_pub_status_result(uint8_t status, const Result &result = Result(), const char *text = "") {
//        printf("ActionServer %s: %s %s -> %s, with result\n", name, goal_status.goal_id.id, strstatus(goal_status.status), strstatus(status));
        goal_status_.status = status;
        goal_status_.text = text;

        _pub_status_array(goal_status_);

        pub_result_msg_.header.seq = count_pub_result_++;
        pub_result_msg_.header.stamp = nh_->now();
        pub_result_msg_.header.frame_id = "";
        pub_result_msg_.status = goal_status_;
        pub_result_msg_.result = result;

        pub_result_.publish(&pub_result_msg_);
    }

    static const char *strstatus(uint8_t status) {
        switch (status) {
        case actionlib_msgs::GoalStatus::PENDING:
            return "PENDING";
        case actionlib_msgs::GoalStatus::ACTIVE:
            return "ACTIVE";
        case actionlib_msgs::GoalStatus::PREEMPTED:
            return "PREEMPTED";
        case actionlib_msgs::GoalStatus::SUCCEEDED:
            return "SUCCEEDED";
        case actionlib_msgs::GoalStatus::ABORTED:
            return "ABORTED";
        case actionlib_msgs::GoalStatus::REJECTED:
            return "REJECTED";
        case actionlib_msgs::GoalStatus::PREEMPTING:
            return "PREEMPTING";
        case actionlib_msgs::GoalStatus::RECALLING:
            return "RECALLING";
        case actionlib_msgs::GoalStatus::RECALLED:
            return "RECALLED";
        case actionlib_msgs::GoalStatus::LOST:
            return "LOST";
        default:
            return "UNKNOWN";
        }
    }

};

// implement

template<class ActionSpec>
ActionServer<ActionSpec>::ActionServer(const char *name,
                                       decltype(goal_callback_) goal_cb, decltype(cancel_callback_) cancel_cb)
        : name_(name), goal_callback_(goal_cb), cancel_callback_(cancel_cb),
          sub_goal_(NULL, std::bind(&ActionServer<ActionSpec>::_on_goal, this, std::placeholders::_1)),
          sub_cancel_(NULL, std::bind(&ActionServer<ActionSpec>::_on_cancel, this, std::placeholders::_1)),
          pub_status_(NULL, &pub_status_msg_),
          pub_feedback_(NULL, &pub_feedback_msg_),
          pub_result_(NULL, &pub_result_msg_) {
    char tmp[128];
    
    snprintf(tmp, sizeof(tmp), "%s/goal", name_);
    sub_goal_name_ = (char*)malloc(strlen(tmp) + 1);
    strcpy(sub_goal_name_, tmp);
    sub_goal_.topic_ = sub_goal_name_;

    snprintf(tmp, sizeof(tmp), "%s/cancel", name_);
    sub_cancel_name_ = (char*)malloc(strlen(tmp) + 1);
    strcpy(sub_cancel_name_, tmp);
    sub_cancel_.topic_ = sub_cancel_name_;

    snprintf(tmp, sizeof(tmp), "%s/status", name_);
    pub_status_name_ = (char*)malloc(strlen(tmp) + 1);
    strcpy(pub_status_name_, tmp);
    pub_status_.topic_ = pub_status_name_;

    snprintf(tmp, sizeof(tmp), "%s/feedback", name_);
    pub_feedback_name_ = (char*)malloc(strlen(tmp) + 1);
    strcpy(pub_feedback_name_, tmp);
    pub_feedback_.topic_ = pub_feedback_name_;

    snprintf(tmp, sizeof(tmp), "%s/result", name_);
    pub_result_name_ = (char*)malloc(strlen(tmp) + 1);
    strcpy(pub_result_name_, tmp);
    pub_result_.topic_ = pub_result_name_;

    goal_status_.status = actionlib_msgs::GoalStatus::SUCCEEDED;
    goal_status_.text = goal_status_text_;
    goal_status_.goal_id.stamp = ros::Time();
    goal_status_.goal_id.id = goal_id_text_;
}

template<class ActionSpec>
void ActionServer<ActionSpec>::start(ros::NodeHandle *_nh) {
    nh_ = _nh;
    nh_->subscribe(sub_goal_);
    nh_->subscribe(sub_cancel_);
    nh_->advertise(pub_status_);
    nh_->advertise(pub_feedback_);
    nh_->advertise(pub_result_);
    printf("ActionServer %s: started\n", name_);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::registerGoalCallback(std::function<void(const Goal &)> goal_cb) {
    goal_callback_ = goal_cb;
}

template<class ActionSpec>
void ActionServer<ActionSpec>::registerCancelCallback(std::function<void()> cancel_cb) {
    cancel_callback_ = cancel_cb;
}

template<class ActionSpec>
const actionlib_msgs::GoalID &ActionServer<ActionSpec>::getGoalID() {
    return goal_status_.goal_id;
}

template<class ActionSpec>
const actionlib_msgs::GoalStatus &ActionServer<ActionSpec>::getGoalStatus() {
    return goal_status_;
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setAborted(const Result &result, const char *text) {
    _change_pub_status_result(actionlib_msgs::GoalStatus::ABORTED, result, text);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setAccepted(const char *text) {
    _change_pub_status(actionlib_msgs::GoalStatus::ACTIVE, text);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setCanceled(const Result &result, const char *text) {
    _change_pub_status_result(actionlib_msgs::GoalStatus::PREEMPTED, result, text);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setRejected(const Result &result, const char *text) {
    _change_pub_status_result(actionlib_msgs::GoalStatus::REJECTED, result, text);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::setSucceeded(const Result &result, const char *text) {
    _change_pub_status_result(actionlib_msgs::GoalStatus::SUCCEEDED, result, text);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::publishFeedBack(const Feedback &feedback) {
    pub_feedback_msg_.header.seq = count_pub_feedback_++;
    pub_feedback_msg_.header.stamp = nh_->now();
    pub_feedback_msg_.header.frame_id = "";
    pub_feedback_msg_.status = goal_status_;
    pub_feedback_msg_.feedback = feedback;
    pub_feedback_.publish(&pub_feedback_msg_);
}

template<class ActionSpec>
void ActionServer<ActionSpec>::update() {
    if (HAL_GetTick() - last_pub_status_ >= pub_status_interval_) {

        if (goal_status_.status == actionlib_msgs::GoalStatus::ACTIVE) {
            _pub_status_array(goal_status_);
        } else {
            _pub_status_array_none();
        }
    }
}

template<class ActionSpec>
void ActionServer<ActionSpec>::_on_goal(const ActionGoal &_action_goal) {

    if (goal_status_.status == actionlib_msgs::GoalStatus::ACTIVE) {
        // goal is active, cancel previous goal
//        printf("ActionServer %s: New goal recved, prevoius goal %s is canceled\n", name, goal_status.goal_id.id);
        if (cancel_callback_) {
            cancel_callback_(*this);
        }
        // status should be SUCCEEDED, ABORTED, PREEMPTED
        if (goal_status_.status != actionlib_msgs::GoalStatus::SUCCEEDED &&
            goal_status_.status != actionlib_msgs::GoalStatus::ABORTED &&
            goal_status_.status != actionlib_msgs::GoalStatus::PREEMPTED) {
            printf("ActionServer %s: Goal %s is not finished after cancel_callback!\n", name_, goal_status_.goal_id.id);
        }
    }
    goal_status_.goal_id.stamp = _action_goal.goal_id.stamp;
    strncpy(goal_id_text_, _action_goal.goal_id.id, goal_id_text_max_length_);
    goal_status_.goal_id.id = goal_id_text_;
    goal_status_.status = actionlib_msgs::GoalStatus::PENDING;
    // new goal change to PENDING
    _pub_status(goal_status_.goal_id, actionlib_msgs::GoalStatus::PENDING);

//    printf("ActionServer %s: Handle new goal %s\n", name, goal_status.goal_id.id);
    if (goal_callback_) {
        goal_callback_(*this, _action_goal.goal);
    }
    // status should be REJECTED, ACTIVE
    if (goal_status_.status != actionlib_msgs::GoalStatus::REJECTED &&
        goal_status_.status != actionlib_msgs::GoalStatus::ACTIVE) {
        printf("ActionServer %s: Goal %s is not handled after goal_callback!\n", name_, goal_status_.goal_id.id);
    }
}

template<class ActionSpec>
void ActionServer<ActionSpec>::_on_cancel(const actionlib_msgs::GoalID &_goal_id) {
    bool need_cancel = false;
    if (strlen(_goal_id.id) > 0 && strcmp(_goal_id.id, goal_status_.goal_id.id) == 0) {
        need_cancel = true;
    } else if (!_goal_id.stamp.isZero() && _goal_id.stamp.toNsec() < goal_status_.goal_id.stamp.toNsec()) {
        need_cancel = true;
    }

    if (need_cancel) {
        if (goal_status_.status == actionlib_msgs::GoalStatus::ACTIVE) {
            _change_pub_status(actionlib_msgs::GoalStatus::PREEMPTING);

            // cancel previous goal
//            printf("ActionServer %s: Cancel goal %s\n", name, goal_status.goal_id.id);
            if (cancel_callback_) {
                cancel_callback_(*this);
            }
            // status should be SUCCEEDED, ABORTED, PREEMPTED
            if (goal_status_.status != actionlib_msgs::GoalStatus::SUCCEEDED &&
                goal_status_.status != actionlib_msgs::GoalStatus::ABORTED &&
                goal_status_.status != actionlib_msgs::GoalStatus::PREEMPTED) {
                printf("ActionServer %s: Goal %s is not finished after cancel_callback!\n", name_,
                       goal_status_.goal_id.id);
            }
        } else {
            // previous goal has been finished
            printf("ActionServer %s: Goal %s has been canceled!\n", name_, goal_status_.goal_id.id);
        }
    }
//    else {
//        // unknown goal
//        printf("ActionServer %s: Unknown goal %s!\n", name, _goal_id.id);
//    }
}

//#include <actionlib/TestAction.h>

//void action_test_update();
//extern ActionServer<actionlib::TestAction> action_server_test;


}