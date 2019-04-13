#pragma once

#include <cassert>
#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <stdexcept>

#include "stm32f4xx_hal.h"

#include "main.h"
#include "usart.h"

#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"
#include "ocservo_driver.hpp"
#include "utils.hpp"
#include "base_motor.hpp"

namespace hustac {

struct OCServoDeviceROBS181 {
#pragma pack(push)    // 记录当前内存对齐
#pragma pack(1)       // 设定为1字节对齐
    struct Register { // 假定CPU为小端序
        uint16_t model;
        uint8_t none1;
        uint16_t firmware_version;
        uint8_t id;
        uint8_t baud_rate;
        uint8_t response_delay;
        uint8_t response_level;
        uint8_t min_position_H;
        uint8_t min_position_L;
        uint8_t max_position_H;
        uint8_t max_position_L;
        uint8_t max_temperature;
        uint8_t max_voltage;
        uint8_t min_voltage;
        uint8_t max_load_H;
        uint8_t max_load_L;
        uint8_t pwm_phase_mode;
        uint8_t unload_flag;
        uint8_t led_alarm_flag;
        uint8_t pid_p;
        uint8_t pid_d;
        uint8_t pid_i;
        uint8_t starting_force_H;
        uint8_t starting_force_L;
        uint8_t dead_zone_cw;
        uint8_t dead_zone_ccw;
        uint16_t pid_integral_limit;
        uint8_t differential_sampling_coefficient;
        uint32_t none2;
        uint32_t none3;
        uint8_t none4;
        uint8_t torque_enable;
        uint8_t none5;
        uint8_t goal_position_H;
        uint8_t goal_position_L;
        uint8_t goal_time_H;
        uint8_t goal_time_L;
        uint8_t goal_velocity_H;
        uint8_t goal_velocity_L;
        uint8_t eeprom_lock;
        uint32_t none6;
        uint16_t none7;
        uint8_t none8;
        uint8_t present_position_H;
        uint8_t present_position_L;
        uint8_t present_velocity_H;
        uint8_t present_velocity_L;
        uint8_t present_load_H;
        uint8_t present_load_L;
        uint8_t present_voltage;
        uint8_t present_temperature;
        uint8_t reg_write_flag;
    } registers;
#pragma pack(pop) // 还原内存对齐
    
    static uint32_t offset_goal_position() { return offsetOf(&Register::goal_position_H); }
    static uint32_t sizeof_goal_position() {
        return sizeof(Register::goal_position_H) + sizeof(Register::goal_position_L);
    }
    
    static uint32_t offset_read_data() { return offsetOf(&Register::present_position_H); }
    static uint32_t sizeof_read_data() { return 8; }
    
    uint8_t *memory() {
        return (uint8_t *) &registers;
    }
    
    static constexpr int position_range_raw = 1024;
    static constexpr float position_range = 2 * (float) M_PI * 330.0f / 360.0f;
    static constexpr float raw_to_rad = position_range / position_range_raw;
    static constexpr int load_range_raw = 1024;
    
    uint8_t dead_zone_cw_raw() {
        return registers.dead_zone_cw;
    }
    int dead_zone_cw_raw(uint8_t val) {
        registers.dead_zone_cw = val;
        return 0;
    }
    uint8_t dead_zone_ccw_raw() {
        return registers.dead_zone_ccw;
    }
    int dead_zone_ccw_raw(uint8_t val) {
        registers.dead_zone_ccw = val;
        return 0;
    }
    uint16_t starting_force_raw() {
        return registers.starting_force_L | ((uint16_t) registers.starting_force_H << 8);
    }
    int starting_force_raw(uint16_t val) {
        if (val >= load_range_raw)
            val = uint16_t(load_range_raw - 1);
        registers.starting_force_H = uint8_t(val >> 8);
        registers.starting_force_L = uint8_t(val);
        return 0;
    }
    
    // 获取电机当前角度, 范围[0, 1023]
    uint16_t present_position_raw() {
        return registers.present_position_L | ((uint16_t) registers.present_position_H << 8);
    }
    // 获取电机当前角度, 单位: 弧度
    float present_position() {
        int pos_raw = present_position_raw() - position_range_raw / 2;
        float pos = pos_raw * raw_to_rad;
        // 转换到[ -position_range/2, position_range/2)
        pos = std::remainder(pos, position_range);
        return pos;
    }
    
    // 获取电机当前速度, bit/s
    int16_t present_velocity_raw() {
        uint16_t raw = (uint16_t) registers.present_velocity_L | ((uint16_t) registers.present_velocity_H << 8);
        int16_t vel;
        if (raw >= 32768) {
            vel = int16_t((int) 32767 - raw);
        } else {
            vel = raw;
        }
        return vel;
    }
    // 计算当前速度, 单位弧度每秒
    float present_velocity() {
        int16_t vel_raw = present_velocity_raw();
        float rad_per_sec = vel_raw * raw_to_rad;
        return rad_per_sec;
    }
    
    // 当前电机占空比, range: [-1024, 1023]
    int16_t present_load_raw() {
        uint16_t present_load = (uint16_t) registers.present_load_L | ((uint16_t) registers.present_load_H << 8);
        int load = present_load;
        if (load >= load_range_raw) {
            load = (int) load_range_raw - 1 - load;
        }
        return (int16_t) load;
    }
    // 当前电机占空比, range: [-1, 1]
    float present_load() {
        return present_load_raw() / (float) load_range_raw;
    }
    
    static constexpr float max_effort = 1.7652;     // unit: Nm (18 kgcm)
    // (估计)当前电机负载, 单位: Nm, range: [-1.7652, 1.7652] (18kgcm)
    float present_effort() {
        return present_load() * max_effort;
    }
    
    // 设置电机目标位置, 范围无限制
    int goal_position(float goal_position) {
        if (!std::isfinite(goal_position)) {
            return -1;
        }
        // 转换到 [-pi, pi)
        goal_position = std::remainder(goal_position, 2 * (float) M_PI);
        // 限幅到 [-165,165) 度
        goal_position = std::min(position_range / 2, goal_position);
        goal_position = std::max(-position_range / 2, goal_position);
        // 转换到 [0, 1023]
        int pos = int(goal_position / raw_to_rad) + position_range_raw / 2;
        
        return goal_position_raw(pos);
    }
    int goal_position_raw(int raw) {
        // 限幅到 [0, 1023]
        uint16_t pos_u16 = (uint16_t) std::min(position_range_raw - 1, std::max(0, raw));
        
        registers.goal_position_L = (uint8_t) pos_u16;
        registers.goal_position_H = uint8_t(pos_u16 >> 8);
        return 0;
    }
    
    // 获取目标位置
    uint16_t goal_position_raw() {
        return (uint16_t) registers.goal_position_L | ((uint16_t) registers.goal_position_H << 8);
    }
    // 获取目标位置
    float goal_position() {
        uint16_t raw = goal_position_raw();
        float pos = (raw - position_range_raw / 2) * raw_to_rad;
        // 转换到 [-pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        return pos;
    }
};

// OCServo舵机驱动程序 (TTL通信版本)

class OCServoTTL : public OCServoDriver<OCServoDeviceROBS181, 2> {
public:
    OCServoTTL(const char *name, DMABuffer_UART &single_wire)
        : OCServoDriver(
        name, single_wire,
//        std::array<const char *, 1>{{"head_yaw_joint"}}) {
        std::array<const char *, 2>{{"gaze_joint1", "gaze_joint2"}}) {
        this->ignore_timeout_ = true;
        this->ignore_busy_ = true;
        this->ignore_protocol_ = true;
//        this->interval_send_ = 1000000;
//        this->rx_resp_timeout_ns_ = 500000;
//        this->rx_interval_timeout_ns_ = 500000;
//        this->dur_loop_ = 1000000;
//        this->timer_loop_.set_interval(this->dur_loop_);
    }
    
    
    void init() {
        OCServoOperation op;
        
        /* 修改ID */
//        {
//            const uint8_t old_id = 2;
//            const uint8_t new_id = 1;
//            static_assert(old_id <= motor_count_ && new_id <= motor_count_, "change id must in motor range");
//            printf("%s: ID %d -> %d\n", name_, old_id, new_id);
//
//            // 解锁ROM
//            devices_[old_id - 1].registers.eeprom_lock = 0;
//            op.id = old_id;
//            op.instruction = OCServoInstructionType::WRITE;
//            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::eeprom_lock);
//            op.reg_length = sizeof(OCServoDeviceROBS181::Register::eeprom_lock);
//            op.on_finish = [this](const OCServoOperation &op) {
//                printf("%s[%d]: eeprom unlocked\n", name_, op.id);
//            };
//            add_operation(op);
//            // 写入ID
//            devices_[old_id - 1].registers.id = new_id;
//            op.instruction = OCServoInstructionType::WRITE;
//            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::id);
//            op.reg_length = sizeof(OCServoDeviceROBS181::Register::id);
//            op.on_finish = [this, new_id, old_id](const OCServoOperation &op) {
//                printf("%s[%d]: id overwrited %d -> %d\n", name_, op.id, old_id, new_id);
//            };
//            add_operation(op);
//            // 锁定ROM
//            devices_[new_id - 1].registers.eeprom_lock = 1;
//            op.id = new_id;
//            op.instruction = OCServoInstructionType::WRITE;
//            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::eeprom_lock);
//            op.reg_length = sizeof(OCServoDeviceROBS181::Register::eeprom_lock);
//            op.on_finish = [this](const OCServoOperation &op) {
//                printf("%s[%d]: eeprom locked\n", name_, op.id);
//                while (true);
//            };
//            add_operation(op);
//        }
        
        
        // 初始化指令
        for (int i = 0; i < motor_count_; i++) {
            op.id = uint8_t(i + 1);
            
            // 初始位置
            devices_[i].goal_position(0);
            
            // 复位
            //            op.instruction = OCServoInstructionType::RESET;
            //            op.reg_addr = 0;
            //            op.reg_length = 0;
            //            op.on_finish = [this](const OCServoOperation& op) {
            //                printf(
            //                    "%s[%d]: Reseted\n",
            //                    name, op.id
            //                );
            //            };
            //            add_operation(op);
            
            // 禁用舵机输出
            devices_[i].registers.torque_enable = 0;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::torque_enable);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::torque_enable);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].registers.pid_p = (uint8_t) 90;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::pid_p);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::pid_p);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].registers.pid_d = (uint8_t) 10;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::pid_d);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::pid_d);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].registers.differential_sampling_coefficient = (uint8_t) 2;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::differential_sampling_coefficient);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::differential_sampling_coefficient);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].dead_zone_cw_raw(0);
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::dead_zone_cw);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::dead_zone_cw);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].dead_zone_ccw_raw(0);
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoDeviceROBS181::Register::dead_zone_ccw);
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::dead_zone_ccw);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            devices_[i].starting_force_raw(10);
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) std::min(offsetOf(&OCServoDeviceROBS181::Register::starting_force_H),
                                             offsetOf(&OCServoDeviceROBS181::Register::starting_force_L));
            op.reg_length = sizeof(OCServoDeviceROBS181::Register::starting_force_H) +
                            sizeof(OCServoDeviceROBS181::Register::starting_force_L);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            // 读取所有寄存器
            op.instruction = OCServoInstructionType::READ;
            op.reg_addr = 0;
            op.reg_length = sizeof(OCServoDeviceROBS181::registers);
            op.on_finish = [this](const OCServoOperation &op) {
                printf(
                    "%s[%d]: Connected, P=%d, I=%d(%d), D=%d(%d), dead=[%d,%d]\n",
                    name_, op.id,
                    devices_[op.id - 1].registers.pid_p,
                    devices_[op.id - 1].registers.pid_i,
                    devices_[op.id - 1].registers.pid_integral_limit,
                    devices_[op.id - 1].registers.pid_d,
                    devices_[op.id - 1].registers.differential_sampling_coefficient,
                    devices_[op.id - 1].registers.dead_zone_cw,
                    devices_[op.id - 1].registers.dead_zone_ccw);
                if (op.id == motor_count_) {
                    on_init_finished();
                }
            };
            add_operation(op);
        }
    }
};

}
