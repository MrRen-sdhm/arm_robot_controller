#pragma once

#include <cstdio>

// ROS
#include <ros.h>
// GPIO输入
#include "button.hpp"
#include "main.h"
// GPIO输出
#include "gpio.hpp"
#include "main.h"
// 串口通信
#include "usart.h"
#include "dmabuffer_uart.hpp"
// 电池
#include "adc.h"
#include "battery.hpp"
// 移动底盘
#include "c620_motor.hpp"
#include "can.h"
#include "macnum_controller.hpp"
#include "ros_adapter_base.hpp"
// 手爪
#include "hand_grip_controller.hpp"
#include "ocservo_rs485.hpp"
#include "ros_adapter_hand.hpp"
// 机械臂
#include "arm_trajectory_controller.hpp"
#include "ros_adapter_arm.hpp"
#include "zeda_motor.hpp"
#include "arm_demo.hpp"
// 升降柱
#include "lift_controller.hpp"
#include "ros_adapter_lift.hpp"
#include "step_motor.hpp"
#include "tim.h"
// 头部
#include "pwm_servo.hpp"
#include "ocservo_ttl.hpp"
#include "ros_adapter_head.hpp"

//#define DISABLE_BATTERY
//#define DISABLE_HAND
#define DISABLE_ARM_RIGHT
//#define DISABLE_ARM_LEFT
//#define DISABLE_BASE
#define DISABLE_ARM_DEMO
#define DISABLE_HEAD
#define DISABLE_LIFT

namespace hustac {

#define NH_CPP_BUF_OUT_SIZE 4096
#define NH_CPP_BUF_IN_SIZE  32768
#define NH_PY_BUF_OUT_SIZE  4096
#define NH_PY_BUF_IN_SIZE   4096
extern uint8_t NH_CPP_BUF_OUT[NH_CPP_BUF_OUT_SIZE];
extern uint8_t NH_CPP_BUF_IN[NH_CPP_BUF_IN_SIZE];
extern uint8_t NH_PY_BUF_OUT[NH_PY_BUF_OUT_SIZE];
extern uint8_t NH_PY_BUF_IN[NH_PY_BUF_IN_SIZE];

class Robot {
public:
    const char* name_;
    SoftTimerMS<1000> timer_debug_ = {0};
    SoftTimerMS<100> timer_100ms_ = {0};
    uint32_t count_main_loop_ = 0;
    uint64_t last_loop_ = 0;
    uint32_t max_loop_interval_ = 0;
    
    uint32_t last_arm_right_count_write_ = 0;
    uint32_t last_arm_right_count_read_ = 0;
    uint32_t last_arm_right_count_error_ = 0;
    uint32_t last_arm_right_count_send_ = 0;
    uint32_t last_arm_left_count_write_ = 0;
    uint32_t last_arm_left_count_read_ = 0;
    uint32_t last_arm_left_count_error_ = 0;
    uint32_t last_arm_left_count_send_ = 0;
    uint32_t last_hand_count_sync_op_ = 0;
    uint32_t last_hand_count_read_ = 0;
    uint32_t last_hand_count_error_ = 0;
    uint32_t last_hand_count_send_ = 0;
    uint32_t last_head_count_sync_op_ = 0;
    uint32_t last_head_count_read_ = 0;
    uint32_t last_head_count_error_ = 0;
    uint32_t last_head_count_send_ = 0;
    uint32_t last_base_count_update_ = 0;
    uint32_t last_base_count_send_ = 0;
    
    // GPIO输入
    // 长按后自动重复
    Button KEY_UP;
    Button KEY_DOWN;
    // 长按后触发长按操作
    Button BUT_START;
    Button BUT_STOP;
    Button BUT_RESET;
    // 长按无特殊操作
    Button BUT_EMG1;
    Button BUT_EMG2;
    // GPIO输出
    GPIOOutput LED1;
    GPIOOutput LED2;
    GPIOOutput BEEP;
    GPIOOutput LED_STATUS1;
    GPIOOutput LED_STATUS2;
    GPIOOutput LED_START;
    GPIOOutput LED_STOP;
    GPIOOutput LED_RESET;
    GPIOOutput LIFT_UNLOCK;
    GPIOOutput LIFT_DISABLE;
    GPIOOutput LIFT_DIR_DOWN;
    GPIOOutput LIFT_PULSE;
    GPIOOutput RS485_1_WE;
    GPIOOutput RS485_2_WE;
    GPIOOutput RS485_3_WE;
    // 串口通信
#define define_full_duplex_with_static_buffer(name, huart, tx_len, rx_len) \
    uint8_t name##_TX_BUF[tx_len] = {0};                           \
    uint8_t name##_RX_BUF[rx_len] = (0);                           \
    DMABuffer_UART name = {#name, huart, name##_TX_BUF, tx_len, name##_RX_BUF, rx_len};
#define define_half_duplex_with_static_buffer(name, huart, write_enable, tx_len, rx_len) \
    uint8_t name##_TX_BUF[tx_len] = {0};                                          \
    uint8_t name##_RX_BUF[rx_len] = {0};                                          \
    DMABuffer_UART name = {#name, huart, write_enable, name##_TX_BUF, tx_len, name##_RX_BUF, rx_len};
    // 注意缓冲区尺寸必须为2的整数倍
    define_half_duplex_with_static_buffer(UART_HEAD, &hsingle_wire, NULL, 128, 128); // 半双工单线TTL: 1M 8N1
    define_half_duplex_with_static_buffer(RS485_ARM_RIGHT, &hrs485_1, &RS485_1_WE, 128,
                                          128); // 半双工RS485(带有输出使能引脚): 1M 8N1
    define_half_duplex_with_static_buffer(RS485_ARM_LEFT, &hrs485_2, &RS485_2_WE, 128,
                                          128); // 半双工RS485(带有输出使能引脚): 1M 8N1
    define_half_duplex_with_static_buffer(RS485_HAND, &hrs485_3, &RS485_3_WE, 128, 128); // 半双工RS485(带有输出使能引脚): 1M 8N1
#undef define_full_duplex_with_static_buffer
#undef define_half_duplex_with_static_buffer
    
    // 电池
    Battery battery_;
    // ROS通信
    ros::NodeHandle nh_cpp_;    // 服务端采用 rosserial_server (cpp实现) 适用于高效率
    ros::NodeHandle nh_py_;     // 服务端采用 rosserial_python (python实现) 功能全面
#ifndef DISABLE_BASE
    C620Motor macnum_motor_;
    MacnumController mobile_base_;
    ROSAdapterBase adapter_base_;
#endif
    // 手爪
#ifndef DISABLE_HAND
    OCServoRS485 hand_servo_;
    HandGripController hand_;
    ROSAdapterHand adapter_hand_;
#endif
    // 机械臂
#ifndef DISABLE_ARM_RIGHT
    ArmMotorT arm_right_;
    ArmControllerT& arm_right_ctrl_;
    ArmAdapterT adapter_arm_right_;
#endif
#ifndef DISABLE_ARM_LEFT
    ArmMotorT arm_left_;
    ArmControllerT &arm_left_ctrl_;
    ArmAdapterT adapter_arm_left_;
#endif
#ifndef DISABLE_ARM_DEMO
    ArmDemo<ArmControllerT> arm_demo_;
#endif
#ifndef DISABLE_LIFT
    // 升降柱
    StepMotor lift_motor_;
    std::array<LiftSensor, 3> lift_sensors_;
    LiftController lift_;
#endif
#ifndef DISABLE_HEAD
    // 头部
    OCServoTTL head_servo_;
    ROSAdapter::HeadIndividual head_adapter_;
#endif
    
    explicit Robot(const char* name)
        : name_(name)
        
        // GPIO输入
        , KEY_UP("KEY_UP", KEY_UP_GPIO_Port, KEY_UP_Pin, GPIO_PIN_RESET, 20, 1000, 100),
          KEY_DOWN("KEY_DOWN", KEY_DOWN_GPIO_Port, KEY_DOWN_Pin, GPIO_PIN_RESET, 20, 1000, 100)
        // 长按后触发长按操作
        , BUT_START("BUT_START", BUT_START_GPIO_Port, BUT_START_Pin, GPIO_PIN_RESET, 20, 1000, 0),
          BUT_STOP("BUT_STOP", BUT_STOP_GPIO_Port, BUT_STOP_Pin, GPIO_PIN_RESET, 20, 1000, 0),
          BUT_RESET("BUT_RESET", BUT_RESET_GPIO_Port, BUT_RESET_Pin, GPIO_PIN_RESET, 20, 1000, 0)
        // 长按无特殊操作
        , BUT_EMG1("BUT_EMG1", BUT_EMG1_GPIO_Port, BUT_EMG1_Pin, GPIO_PIN_SET, 20, 0, 0),
          BUT_EMG2("BUT_EMG2", BUT_EMG2_GPIO_Port, BUT_EMG2_Pin, GPIO_PIN_SET, 20, 0, 0)
        
        // GPIO输出
#define define_gpio_output(name, active_state) \
    name(#name, name##_GPIO_Port, name##_Pin, active_state)
        , define_gpio_output(LED1, GPIO_PIN_RESET), define_gpio_output(LED2, GPIO_PIN_RESET),
          define_gpio_output(LED_STATUS1, GPIO_PIN_SET), define_gpio_output(LED_STATUS2, GPIO_PIN_SET),
          define_gpio_output(LED_START, GPIO_PIN_SET), define_gpio_output(LED_STOP, GPIO_PIN_SET),
          define_gpio_output(LED_RESET, GPIO_PIN_SET), define_gpio_output(BEEP, GPIO_PIN_SET),
          define_gpio_output(LIFT_UNLOCK, GPIO_PIN_RESET) // on为解锁, off为锁定
        , define_gpio_output(LIFT_DISABLE, GPIO_PIN_SET) // on为禁用电机, off为启用电机
        , define_gpio_output(LIFT_DIR_DOWN, GPIO_PIN_SET) // on为下降, off为上升
        , define_gpio_output(LIFT_PULSE, GPIO_PIN_SET) // on为有效电平(高电平), off为无效电平(低电平)
        , define_gpio_output(RS485_1_WE, GPIO_PIN_SET), define_gpio_output(RS485_2_WE, GPIO_PIN_SET),
          define_gpio_output(RS485_3_WE, GPIO_PIN_SET)
#undef define_gpio_output
        
        // ROS
        , nh_cpp_("CPP"), nh_py_("PY")
        // 电池
        , battery_("BAT", &hadc1, &hadc2)
        // 移动底盘
#ifndef DISABLE_BASE
        , macnum_motor_("MACNUM_MOTOR", &hcan1), mobile_base_("BASE", macnum_motor_),
          adapter_base_("ROS-BASE", nh_cpp_, mobile_base_, "base/cmd_vel", "base/joint_states", "base/odom")
#endif
        // 手爪
#ifndef DISABLE_HAND
    , hand_servo_("HAND_SERVO", RS485_HAND), hand_("HAND", hand_servo_)
    , adapter_hand_("ROS-HAND", nh_cpp_, hand_, "hand/joint_states")
#endif
        // 机械臂
#ifndef DISABLE_ARM_RIGHT
        , arm_right_("ARM_RIGHT", RS485_ARM_RIGHT, "right_joint"),
          arm_right_ctrl_(*new(ARM_RIGHT_CTRL_MEM_REGION) ArmControllerT("ARM_RIGHT_CTRL", arm_right_)),
          adapter_arm_right_("ROS-ARM-R", nh_cpp_, arm_right_ctrl_, "arm/right/follow_joint_trajectory",
                             "arm/right/joint_states")
#endif
#ifndef DISABLE_ARM_LEFT
        , arm_left_("ARM_LEFT", RS485_ARM_LEFT, "left_joint"),
          arm_left_ctrl_(*new(ARM_LEFT_CTRL_MEM_REGION) ArmControllerT("ARM_LEFT_CTRL", arm_left_)),
          adapter_arm_left_("ROS-ARM-L", nh_cpp_, arm_left_ctrl_, "arm/left/follow_joint_trajectory",
                            "arm/left/joint_states")
#endif
#ifndef DISABLE_ARM_DEMO
        , arm_demo_(arm_right_ctrl_)
#endif
        // 升降柱
#ifndef DISABLE_LIFT
    , lift_sensors_{
        LiftSensor("BOTTOM", SENSE_BOTTOM_GPIO_Port, SENSE_BOTTOM_Pin),
        LiftSensor("MIDDLE", SENSE_MIDDLE_GPIO_Port, SENSE_MIDDLE_Pin),
        LiftSensor("TOP", SENSE_TOP_GPIO_Port, SENSE_TOP_Pin),
    }, lift_motor_("LIFT_MOTOR", &htim3, TIM_CHANNEL_2, LIFT_DISABLE, LIFT_UNLOCK, LIFT_DIR_DOWN,
                   LIFT_PULSE), lift_("LIFT", lift_sensors_, lift_motor_)
#endif
#ifndef DISABLE_HEAD
        // 头部
        , head_servo_("HEAD_SERVO", UART_HEAD),
          head_adapter_("HEAD_ADAPTER", nh_cpp_, head_servo_, "gaze/joint_states", "gaze/single_joint_position/action",
                        "gaze/single_joint_position/direct")
#endif
    
    {
    }
    
    // 执行不宜在全局初始化中执行的初始化步骤
    void init();
    
    void emergency_stop(bool enable);
    
    void spin_once();

private:
    // 处理按键事件
    void handle_button();
    // 处理串口指令
    void handle_command();
};

inline void Robot::init() {
    DMABuffer_UART::init_uart();
    
    printf("\n\n=================== ARM ROBOT - %s ===================\n", name_);
    timer_debug_.reset();
    timer_100ms_.reset();
    
    // GPIO输入
    Button::init_button();
    // GPIO输出
    GPIOOutput::init_pin();
    LED1.on();
    LED2.off();
    LED_START.on();
    LED_STOP.off();
    
    // 电池
#ifndef DISABLE_BATTERY
    battery_.init();
#endif
    // ROS 通信
    nh_cpp_.initNode("11412", NH_CPP_BUF_IN, sizeof(NH_CPP_BUF_IN), NH_CPP_BUF_OUT, sizeof(NH_CPP_BUF_OUT));
    nh_py_.initNode("11411", NH_PY_BUF_IN, sizeof(NH_PY_BUF_IN), NH_PY_BUF_OUT, sizeof(NH_PY_BUF_OUT));
    // 移动底盘
#ifndef DISABLE_BASE
    macnum_motor_.init();
    //    mobile_base_.init();
    adapter_base_.init();
#endif
    // 手爪
#ifndef DISABLE_HAND
    hand_servo_.init();
    //    hand_.init();
    adapter_hand_.init();
#endif
    // 机械臂
#ifndef DISABLE_ARM_RIGHT
    //    arm_right_.init();
    //    arm_right_ctrl_.init();
    adapter_arm_right_.init();
#endif
#ifndef DISABLE_ARM_LEFT
    //    arm_left_.init();
    //    arm_left_ctrl_.init();
    adapter_arm_left_.init();
#endif
#ifndef DISABLE_LIFT
    // 升降柱
    lift_.init();
#endif
#ifndef DISABLE_HEAD
    head_servo_.init();
    head_adapter_.init();
#endif
}

inline void Robot::emergency_stop(bool enable) {
//    mobile_base_.emergency_stop(enable);
//    hand_.emergency_stop(enable);
//    arm_right_ctrl_.set_emergency_stop(enable);
//    arm_left_ctrl_.set_emergency_stop(enable);
    EmergencyStoppable::all_emergency_stop(enable);
    if (enable) {
        LED_START.off();
        LED_STOP.on();
    } else {
        LED_START.on();
        LED_STOP.off();
    }
}

inline void Robot::spin_once() {
    if (timer_100ms_.is_timeout()) {
        LED_RESET.toggle();
        LED1.toggle();
        LED2.toggle();
    }
    
    uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
    // 计算主循环最大时间间隔
    if (last_loop_ && now - last_loop_ > max_loop_interval_) {
        max_loop_interval_ = uint32_t(now - last_loop_);
    }
    last_loop_ = now;
    count_main_loop_++;
    if (timer_debug_.is_timeout()) {
        printf(
            "[%s%s-%lu%s]: %d-%d",
            nh_cpp_.connected() ? "T" : "F",
            nh_py_.connected() ? "T" : "F",
            timer_debug_.count_, EmergencyStoppable::all_emergency_stop() ? "-STOP" : "",
            int(1000000 / count_main_loop_),
            int(max_loop_interval_ / 1000));
#ifndef DISABLE_BATTERY
        // battery
        printf(" %.2fV", battery_.voltage());
#endif
#ifndef DISABLE_ARM_RIGHT
        // arm_right
        printf(" ARM_R-W%lu-R%lu-S%lu-E%lu",
               arm_right_.count_sync_op_ - last_arm_right_count_write_,
               arm_right_.count_read_ - last_arm_right_count_read_,
               adapter_arm_right_.state_publisher_->count_pub_joint_ - last_arm_right_count_send_,
               arm_right_.count_error_ - last_arm_right_count_error_);
        last_arm_right_count_write_ = arm_right_.count_sync_op_;
        last_arm_right_count_read_ = arm_right_.count_read_;
        last_arm_right_count_error_ = arm_right_.count_error_;
        last_arm_right_count_send_ = adapter_arm_right_.state_publisher_->count_pub_joint_;
#endif
#ifndef DISABLE_ARM_LEFT
        // arm_left
        printf(" ARM_L-W%lu-R%lu-S%lu-E%lu",
               arm_left_.count_sync_op_ - last_arm_left_count_write_,
               arm_left_.count_read_ - last_arm_left_count_read_,
               adapter_arm_left_.state_publisher_->count_pub_joint_ - last_arm_left_count_send_,
               arm_left_.count_error_ - last_arm_left_count_error_);
        last_arm_left_count_write_ = arm_left_.count_sync_op_;
        last_arm_left_count_read_ = arm_left_.count_read_;
        last_arm_left_count_error_ = arm_left_.count_error_;
        last_arm_left_count_send_ = adapter_arm_left_.state_publisher_->count_pub_joint_;
#endif
#ifndef DISABLE_HAND
        // hand
        printf(" HAND-W%lu-R%lu-S%lu-E%lu",
               hand_servo_.count_sync_op_ - last_hand_count_sync_op_,
               hand_servo_.count_op_ - last_hand_count_read_,
               adapter_hand_.count_pub_joint_ - last_hand_count_send_,
               hand_servo_.count_error_ - last_hand_count_error_);
        last_hand_count_sync_op_ = hand_servo_.count_sync_op_;
        last_hand_count_read_ = hand_servo_.count_op_;
        last_hand_count_error_ = hand_servo_.count_error_;
        last_hand_count_send_ = adapter_hand_.count_pub_joint_;
#endif
#ifndef DISABLE_HEAD
        // head
        printf(" HEAD-W%lu-R%lu-S%lu-E%lu",
               head_servo_.count_sync_op_ - last_head_count_sync_op_,
               head_servo_.count_read_ - last_head_count_read_,
               head_adapter_.state_publisher_->count_pub_joint_ - last_head_count_send_,
               head_servo_.count_error_ - last_head_count_error_);
        last_head_count_sync_op_ = head_servo_.count_sync_op_;
        last_head_count_read_ = head_servo_.count_read_;
        last_head_count_error_ = head_servo_.count_error_;
        last_head_count_send_ = head_adapter_.state_publisher_->count_pub_joint_;
#endif
#ifndef DISABLE_BASE
        // base
        printf(" BASE-U%lu-S%lu",
               mobile_base_.count_update_ - last_base_count_update_,
               adapter_base_.timer_pub_joint_.count_ - last_base_count_send_);
        last_base_count_update_ = mobile_base_.count_update_;
        last_base_count_send_ = adapter_base_.timer_pub_joint_.count_;
#endif
        
        puts("");
        
        count_main_loop_ = 0;
        max_loop_interval_ = 0;

//        BEEP.toggle();
//        LED1.toggle();
//        LED2.toggle();
//        LED_START.toggle();
//        LED_STOP.toggle();
//        LED_RESET.toggle();
//        LED_STATUS1.toggle();
//        LED_STATUS2.toggle();

//        char buf[65];
//        int ret = RS485_HAND.readsome((uint8_t *) buf, 64);
//        if (ret > 0) {
//            printf("Recved %d\n", ret);
//            buf[ret] = 0;
//            puts(buf);
//        }
//        ret = RS485_ARM_RIGHT.write_string("test rs485: hello world");
//        printf("Sent %d\n", ret);
    }
    
    // GPIO输入
    Button::update_button();
    handle_button();
    // 电池
#ifndef DISABLE_BATTERY
    battery_.spin_once();
#endif
    // ROS 通信
    nh_cpp_.spinOnce();
    nh_py_.spinOnce();
    // 文本指令
    handle_command();
    // 移动底盘
#ifndef DISABLE_BASE
    //    macnum_motor_.spin_once();
    mobile_base_.spin_once();
    adapter_base_.spin_once();
#endif
    // 手爪
#ifndef DISABLE_HAND
    hand_servo_.spin_once();
    //    hand_.spin_once();
    adapter_hand_.spin_once();
#endif
    // 机械臂
#ifndef DISABLE_ARM_RIGHT
    arm_right_.spin_once();
    //    arm_right_ctrl_.spin_once();
    adapter_arm_right_.spin_once();
#ifndef DISABLE_ARM_DEMO
    arm_demo_.spin_once();
#endif
#endif
#ifndef DISABLE_ARM_LEFT
    arm_left_.spin_once();
    //    arm_left_ctrl_.spin_once();
    adapter_arm_left_.spin_once();
#endif
#ifndef DISABLE_LIFT
    // 升降柱
    lift_.update();
#endif
#ifndef DISABLE_HEAD
    head_servo_.spin_once();
    head_adapter_.spin_once();
#endif
}

// 处理按键事件
inline void Robot::handle_button() {
    while (KEY_UP.count_key_press) {
        printf("KEY_UP pressed\n");
        KEY_UP.count_key_press--;

//        lift_.manual_move(1, 0.02);
#ifndef DISABLE_ARM_DEMO
        arm_demo_.start();
#endif
    }
    while (KEY_DOWN.count_key_press) {
        printf("KEY_DOWN pressed\n");
        KEY_DOWN.count_key_press--;

//        lift_.manual_move(-1, 0.02);
#ifndef DISABLE_ARM_DEMO
        arm_demo_.stop();
#endif
    }
    while (BUT_START.count_key_press) {
        printf("BUT_START pressed\n");
        BUT_START.count_key_press--;
    }
    while (BUT_START.count_key_hold) {
        printf("BUT_START long pressed\n");
        BUT_START.count_key_hold--;
    }
    while (BUT_STOP.count_key_press) {
        printf("BUT_STOP pressed\n");
        BUT_STOP.count_key_press--;
    }
    while (BUT_STOP.count_key_hold) {
        printf("BUT_STOP long pressed\n");
        BUT_STOP.count_key_hold--;
    }
    while (BUT_RESET.count_key_press) {
        printf("BUT_RESET pressed\n");
        BUT_RESET.count_key_press--;
    }
    while (BUT_RESET.count_key_hold) {
        printf("BUT_RESET long pressed\n");
        BUT_RESET.count_key_hold--;
    }
    if (BUT_EMG1.is_pressed() && BUT_EMG1.count_key_down) {
        printf("BUT_EMG1 down\n");
        BUT_EMG1.count_key_down = 0;
    }
    if (!BUT_EMG1.is_pressed() && BUT_EMG1.count_key_up) {
        printf("BUT_EMG1 up\n");
        BUT_EMG1.count_key_up = 0;
    }
    if (BUT_EMG2.is_pressed() && BUT_EMG2.count_key_down) {
        printf("BUT_EMG2 down\n");
        BUT_EMG2.count_key_down = 0;
    }
    if (!BUT_EMG2.is_pressed() && BUT_EMG2.count_key_up) {
        printf("BUT_EMG2 up\n");
        BUT_EMG2.count_key_up = 0;
    }
    
    if (BUT_EMG1.is_pressed() || BUT_EMG2.is_pressed()) {
        // 急停状态
        emergency_stop(true);
    } else {
        emergency_stop(false);
    }
}

// 处理指令
inline void Robot::handle_command() {
    while (true) {
        char cmd[128] = "";
        // get cmd
        int ret = fread(cmd, 1, sizeof(cmd) - 1, stdin);
        if (ret < 0) {
            break;
        }
        int cmd_size = ret;
        if (cmd_size > 0) {
            // process cmd
            int arg_1;
            float arg_float;
            if (strcmp(cmd, "ping") == 0) {
                printf("[CMD]: pong\n");
                
            } else if (strcmp(cmd, "help") == 0) {
                printf(
                    "[CMD]: Available command: ping, help, reset, pos, demo start, demo stop, save %%d, load %%d, execute %%d, pitch %%f\n");
                
            } else if (strcmp(cmd, "reset") == 0) {
//                printf("[CMD]: Reset!!!\n");
                NVIC_SystemReset();

//#ifndef DISABLE_ARM_LEFT
//                } else if (strcmp(cmd, "pos") == 0) {
//                    // 打印机械臂状态
//                    arm_right_.print();
//#endif
#ifndef DISABLE_ARM_DEMO
                } else if (strcmp(cmd, "demo start") == 0) {
                    // 机械臂Demo开始
                    arm_demo_.start();
    
                } else if (strcmp(cmd, "demo stop") == 0) {
                    // 机械臂Demo停止
                    arm_demo_.stop();
#endif
//#ifndef DISABLE_ARM_LEFT
//                } else if (sscanf(cmd, "save %d", &arg_1) == 1) {
//                    printf("[CMD]: Saving last arm trajectory to flash (page %d)... ", arg_1);
//                    int ret = arm_right_ctrl_.current_trajectory_.save(arg_1);
//                    printf("%d\n", ret);
//                    arm_right_ctrl_.current_trajectory_.print();
//
//                } else if (sscanf(cmd, "load %d", &arg_1) == 1) {
//                    printf("[CMD]: Loading arm trajectory from flash (page %d)... ", arg_1);
//                    int ret = arm_right_ctrl_.current_trajectory_.load(arg_1);
//                    printf("%d\n", ret);
//                    arm_right_ctrl_.current_trajectory_.print();
//
//                } else if (sscanf(cmd, "execute %d", &arg_1) == 1) {
//                    printf("[CMD]: Execute arm trajectory from flash (page %d)...\n", arg_1);
//                    int ret = arm_right_ctrl_.set_goal(arg_1);
//                    printf("[CMD]: Execute result: %d\n", ret);
//
//#endif
            } else if (sscanf(cmd, "pitch %f", &arg_float) == 1) {
                float rad = arg_float / 180.0f * float(M_PI);
                printf("[CMD]: Head joint pitch Goal=%.3f deg (%.3f rad)\n", arg_float, rad);
//                servo_pitch_ctrl.goal_position(rad);
                
            } else {
                printf("[CMD]: Unknown cmd: ");
                puts(cmd);
            }
        } else {
            // no cmd
            break;
        }
    }
}

extern uint8_t robot_mem_region[sizeof(Robot)];
}
