#pragma once

#include <functional>
#include <stdexcept>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "dmabuffer_uart.hpp"
#include "macnum_controller.hpp"

namespace hustac {

/* 均值滤波队列
 * 由于传感器更新速度, 高于数据发布速度. 采用积分均值算法滤波, 避免混叠*/
template <typename T, size_t max_size>
class AverageQueue {
public:
    uint8_t memory_[sizeof(T) * max_size] = {0};
    T *data_ = (T *) memory_;
    size_t size_ = 0;
    T sum_ = {};
    
    const T& back() const {
        if (size_ <= 0) {
            throw std::out_of_range("AverageQueue is empty()");
        }
        return data_[(size_ - 1) % max_size];
    }
    
    void push(T& v) {
        if (size_ < max_size) {
            new(&data_[size_]) T(v);
            size_++;
            sum_ += v;
        } else {
            sum_ -= data_[size_ % max_size];
            data_[size_ % max_size].~T();
            new(&data_[size_ % max_size]) T(v);
            size_++;
            sum_ += v;
        }
    }
    
    // 队列是否溢出, 返回溢出数量
    int overflow() const {
        if (size_ > max_size) {
            return size_ - max_size;
        } else {
            return 0;
        }
    }
    
    T average() const {
        T avg;
        if (size_ <= 0) {
            avg = {};
        } else {
            size_t count = std::min(size_, max_size);
            avg = sum_ / count;
        }
        return avg;
    }
    
    void clear() {
        for (auto i = 0; i < std::min(size_, max_size); i++) {
            data_[i].~T();
        }
        size_ = 0;
        sum_ = {};
    }
    
    bool empty() const {
        return size_ <= 0;
    }
    
    ~AverageQueue() {
        clear();
    }
};

class ROSAdapterBase {
public:
    const std::array<const char*, 4> joint_names_ = {"lf_wheel_joint", "lb_wheel_joint", "rb_wheel_joint", "rf_wheel_joint"};
    static const int interval_pub_joint_ = 10; // 每隔若干毫秒发送一次关节消息
    static const int interval_pub_odom_ = 10; // 每隔若干毫秒发送一次里程计消息
    static const int timeout_twist_ = 200; // 速度控制指令超时, 超过若干毫秒未接收到速度指令, 则停机
    
    // 关节状态信息, 定义如何积分, 如何求均值
    struct JointStatus {
        uint64_t t = 0;
        std::array<float, 4> positions = {};
        std::array<float, 4> velocities = {};
        std::array<float, 4> efforts = {};
        JointStatus& operator +=(const JointStatus& rhs) {
            if (t < rhs.t) {
                positions = rhs.positions;
                t = rhs.t;
            }
            for (int i = 0; i < 4; i++) {
                velocities[i] += rhs.velocities[i];
                efforts[i] += rhs.efforts[i];
            }
            return *this;
        }
        JointStatus& operator -=(const JointStatus& rhs) {
            if (t < rhs.t) {
                positions = rhs.positions;
                t = rhs.t;
            }
            for (int i = 0; i < 4; i++) {
                velocities[i] -= rhs.velocities[i];
                efforts[i] -= rhs.efforts[i];
            }
            return *this;
        }
        template <typename T>
        friend JointStatus operator /(const JointStatus& lhs, const T& rhs) {
            JointStatus result;
            if (rhs == 0) {
                return result;
            }
            result.t = lhs.t;
            result.positions = lhs.positions;
            for (int i = 0; i < 4; i++) {
                result.velocities[i] = lhs.velocities[i] / rhs;
                result.efforts[i] = lhs.efforts[i] / rhs;
            }
            return result;
        }
        
    };
    // 里程计状态信息, 定义如何积分, 如何求均值
    struct OdometryStatus {
        uint64_t t = 0;
        Twist2D twist = {};
        float constraint_error_vel = 0;
        float pose_x = 0;
        float pose_y = 0;
        float pose_w = 0;
        float constraint_error = 0;
        
        OdometryStatus& operator +=(const OdometryStatus& rhs) {
            if (t < rhs.t) {
                t = rhs.t;
                pose_x = rhs.pose_x;
                pose_y = rhs.pose_y;
                pose_w = rhs.pose_w;
                constraint_error = rhs.constraint_error;
            }
            twist.linear_x += rhs.twist.linear_x;
            twist.linear_y += rhs.twist.linear_y;
            twist.angular += rhs.twist.angular;
            constraint_error_vel += rhs.constraint_error_vel;
            return *this;
        }
        OdometryStatus& operator -=(const OdometryStatus& rhs) {
            if (t < rhs.t) {
                t = rhs.t;
                pose_x = rhs.pose_x;
                pose_y = rhs.pose_y;
                pose_w = rhs.pose_w;
                constraint_error = rhs.constraint_error;
            }
            twist.linear_x -= rhs.twist.linear_x;
            twist.linear_y -= rhs.twist.linear_y;
            twist.angular -= rhs.twist.angular;
            constraint_error_vel -= rhs.constraint_error_vel;
            return *this;
        }
        template <typename T>
        friend OdometryStatus operator /(const OdometryStatus& lhs, const T& rhs) {
            OdometryStatus result;
            if (rhs == 0) {
                return result;
            }
            result.t = lhs.t;
            result.pose_x = lhs.pose_x;
            result.pose_y = lhs.pose_y;
            result.pose_w = lhs.pose_w;
            result.constraint_error = lhs.constraint_error;
            result.twist.linear_x = lhs.twist.linear_x / rhs;
            result.twist.linear_y = lhs.twist.linear_y / rhs;
            result.twist.angular = lhs.twist.angular / rhs;
            result.constraint_error_vel = lhs.constraint_error_vel;
            return result;
        }
    };
    
    const char* name_;
    MacnumController& controller_;
    ros::NodeHandle& nh_;
    // 速度指令
    ros::Subscriber<geometry_msgs::Twist> sub_twist_;
    uint32_t last_recv_twist_ = 0;
    SoftTimerMS<200> timer_print_twist_ = {0};
    bool need_print_twist_ = false;
    
    // 读取关节电机, 里程计状态
    uint32_t last_read_status_ = 0;
    AverageQueue<JointStatus, 16> joint_status_buffers_;
    AverageQueue<OdometryStatus, 16> odometry_status_buffers_;
    
    // 关节状态发布
    sensor_msgs::JointState msg_joint_;
    ros::Publisher pub_joint_;
    SoftTimerMS<interval_pub_joint_> timer_pub_joint_;
    
    // 里程计状态
    nav_msgs::Odometry msg_odom_;
    ros::Publisher pub_odom_;
    // TF 发布
    geometry_msgs::TransformStamped msg_tf_;
    tf::TransformBroadcaster brd_tf_;
    SoftTimerMS<interval_pub_odom_> timer_pub_odom_tf_;
    
    ROSAdapterBase(const char* name, ros::NodeHandle& nh, MacnumController& controller, const char* topic_twist,
                   const char* topic_joint, const char* topic_odom)
        : name_(name), nh_(nh), controller_(controller),
          sub_twist_(topic_twist, std::bind(&ROSAdapterBase::on_twist, this, std::placeholders::_1)),
          pub_joint_(topic_joint, &msg_joint_), pub_odom_(topic_odom, &msg_odom_) {
    }
    
    void init();
    
    void spin_once();

private:
    void on_twist(const geometry_msgs::Twist& msg);
    void print_current_twist();
};

inline void ROSAdapterBase::init() {
    nh_.advertise(pub_joint_);
    nh_.advertise(pub_odom_);
    nh_.subscribe(sub_twist_);
    brd_tf_.init(nh_);
    
    msg_joint_.name_length = 4;
    msg_joint_.position_length = 4;
    msg_joint_.velocity_length = 4;
    msg_joint_.effort_length = 4;
    
    msg_odom_.header.frame_id = "odom";
    msg_odom_.child_frame_id = "base_footprint";
    msg_odom_.twist.twist.linear.z = 0;
    msg_odom_.twist.twist.angular.x = 0;
    msg_odom_.twist.twist.angular.y = 0;
    msg_odom_.twist.covariance[0] = -1;
    for (int i = 1; i < 36; i++) {
        msg_odom_.twist.covariance[i] = 0;
    }
    msg_odom_.pose.pose.position.z = 0;
    msg_odom_.pose.pose.orientation.x = 0;
    msg_odom_.pose.pose.orientation.y = 0;
    msg_odom_.pose.covariance[0] = -1;
    for (int i = 1; i < 36; i++) {
        msg_odom_.pose.covariance[i] = 0;
    }
    
    msg_tf_.header.frame_id = "odom";
    msg_tf_.child_frame_id = "base_footprint";
    msg_tf_.transform.translation.z = 0;
    msg_tf_.transform.rotation.x = 0;
    msg_tf_.transform.rotation.y = 0;
}

inline void ROSAdapterBase::spin_once() {
    // 读取关节电机, 里程计状态
    if (last_read_status_ != controller_.count_update_) {
        last_read_status_ = controller_.count_update_;
        JointStatus joint_status;
        for (int i = 0; i < 4; i++) {
            joint_status.t = controller_.motor_.last_all_recved_;
            joint_status.positions[i] = controller_.motor_[i].position;
            joint_status.velocities[i] = controller_.motor_[i].velocity;
            joint_status.efforts[i] = controller_.motor_[i].effort;
        }
        joint_status_buffers_.push(joint_status);
        OdometryStatus odometry_status = {
            .t = controller_.motor_.last_all_recved_,
            .twist = controller_.actual_velocity(),
            .constraint_error_vel = controller_.constraint_error_velocity_,
            .pose_x = controller_.odometry().x(),
            .pose_y = controller_.odometry().y(),
            .pose_w = controller_.odometry().angle(),
            .constraint_error = controller_.odometry().constraint_error()
        };
        odometry_status_buffers_.push(odometry_status);
    }
    
    // 定时发布关节状态
    timer_pub_joint_.is_timeout([this](uint32_t count) {
        if (joint_status_buffers_.empty()) {
            // 没有新的关节状态信息, 取消发布
            return false;
        }
        // 关节消息
        JointStatus joint_status = joint_status_buffers_.average();
        msg_joint_.header.seq = count;
        msg_joint_.header.stamp = nh_.fromNSec(joint_status.t);
        msg_joint_.name = (char**) joint_names_.data();
        msg_joint_.position = joint_status.positions.data();
        msg_joint_.velocity = joint_status.velocities.data();
        msg_joint_.effort = joint_status.efforts.data();
        // 发送消息
        if (pub_joint_.publish(&msg_joint_) < 0) {
            printf("%s: failed to publish msg_joint_\n", name_);
            return false;
        } else {
            joint_status_buffers_.clear();
            return true;
        }
    });
    
    // 定时发布里程计状态, tf消息
    timer_pub_odom_tf_.is_timeout([this](uint32_t count) {
        if (odometry_status_buffers_.empty()) {
            // 没有新的里程计数据, 取消发布
            return false;
        }
    
        bool ret = false;
        OdometryStatus odom_status = odometry_status_buffers_.average();
        float q_z = std::sin(odom_status.pose_w / 2.0f);
        float q_w = std::cos(odom_status.pose_w / 2.0f);
        
//        printf("%s: %.3f deg/s\n", name_, controller_.actual_velocity_.angular / float(M_PI) * 180);
        // 里程计消息
        msg_odom_.header.seq = count;
        msg_odom_.header.stamp = nh_.fromNSec(odom_status.t);
        msg_odom_.twist.twist.linear.x = odometry_status_buffers_.back().twist.linear_x;
        msg_odom_.twist.twist.linear.y = odometry_status_buffers_.back().twist.linear_y;
        msg_odom_.twist.twist.angular.z = odometry_status_buffers_.back().twist.angular;
        // 使用协方差矩阵中的第二个元素记录运动约束误差
        msg_odom_.twist.covariance[1] = odometry_status_buffers_.back().constraint_error_vel;
        msg_odom_.pose.pose.position.x = odom_status.pose_x;
        msg_odom_.pose.pose.position.y = odom_status.pose_y;
        msg_odom_.pose.pose.orientation.z = q_z;
        msg_odom_.pose.pose.orientation.w = q_w;
        // 使用协方差矩阵中的第二个元素记录运动约束误差
        msg_odom_.pose.covariance[1] = odom_status.constraint_error;
        // 发送消息
        if (pub_odom_.publish(&msg_odom_) < 0) {
            printf("%s: failed to publish msg_odom_\n", name_);
            ret |= false;
        } else {
            odometry_status_buffers_.clear();
            ret |= true;
        }
        
        // tf消息
        msg_tf_.header.seq = count;
        msg_tf_.header.stamp = nh_.fromNSec(odom_status.t);
        msg_tf_.transform.translation.x = odom_status.twist.linear_x;
        msg_tf_.transform.translation.y = odom_status.twist.linear_y;
        msg_tf_.transform.rotation.z = q_z;
        msg_tf_.transform.rotation.w = q_w;
        // 发送消息
        if (brd_tf_.sendTransform(msg_tf_) < 0) {
            printf("%s: failed to broadcast tf %s -> %s\n", name_, msg_tf_.header.frame_id, msg_tf_.child_frame_id);
            ret |= false;
        } else {
            ret |= true;
        }
        return ret;
    });
    
    // 检查速度指令是否超时
    uint32_t now_ms = HAL_GetTick();
    if (last_recv_twist_ != 0 && now_ms - last_recv_twist_ >= timeout_twist_) {
        // 距离上一次接收到速度指令已超时, 停车
        controller_.desired_velocity(Twist2D{.linear_x = 0, .linear_y = 0, .angular = 0});
        printf("%s: Twist timeout(%ld ms)! Stop\n", name_, now_ms - last_recv_twist_);
        last_recv_twist_ = 0;
        need_print_twist_ = true;
    }
    
    print_current_twist();
}

inline void ROSAdapterBase::on_twist(const geometry_msgs::Twist& msg) {
    last_recv_twist_ = HAL_GetTick();
    if (msg.linear.z != 0) {
        printf("%s: Twist.linear.z != 0\n", name_);
    }
    if (msg.angular.x != 0) {
        printf("%s: Twist.angular.x != 0\n", name_);
    }
    if (msg.angular.y != 0) {
        printf("%s: Twist.angular.y != 0\n", name_);
    }
    if (!std::isfinite(msg.linear.x)) {
        printf("%s: Twist.linear.x is not finite\n", name_);
        return;
    }
    if (!std::isfinite(msg.linear.y)) {
        printf("%s: Twist.linear.y is not finite\n", name_);
        return;
    }
    if (!std::isfinite(msg.angular.z)) {
        printf("%s: Twist.angular.z is not finite\n", name_);
        return;
    }
    Twist2D twist;
    twist.linear_x = msg.linear.x;
    twist.linear_y = msg.linear.y;
    twist.angular = msg.angular.z;
    auto old_desired_velocity = controller_.desired_velocity();
    controller_.desired_velocity(twist);
    auto new_desired_velocity = controller_.desired_velocity();
    if (old_desired_velocity.linear_x != new_desired_velocity.linear_x ||
        old_desired_velocity.linear_y != new_desired_velocity.linear_y ||
        old_desired_velocity.angular != new_desired_velocity.angular) {
        need_print_twist_ = true;
        print_current_twist();
    }
}

inline void ROSAdapterBase::print_current_twist() {
    timer_print_twist_.is_timeout([this](uint32_t count) -> bool {
        if (need_print_twist_) {
            auto twist = controller_.desired_velocity();
            printf("%s: Twist=(%.2f m, %.2f m, %.2f deg)/s\n", name_, twist.linear_x, twist.linear_y,
                   twist.angular / float(M_PI) * 180);
            need_print_twist_ = false;
            return true;
        } else {
            return false;
        }
    });
}

}
