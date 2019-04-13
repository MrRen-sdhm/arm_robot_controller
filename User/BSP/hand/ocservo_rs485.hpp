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

namespace hustac {

struct OCServoRS485Device {

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
        uint16_t min_position;
        uint16_t max_position;
        uint8_t max_temperature;
        uint8_t max_voltage;
        uint8_t min_voltage;
        uint16_t max_load;
        uint8_t pwm_phase_mode;
        uint8_t unload_flag;
        uint8_t led_alarm_flag;
        uint8_t pid_p;
        uint8_t pid_d;
        uint8_t pid_i;
        uint16_t starting_force;
        uint8_t dead_zone_cw;
        uint8_t dead_zone_ccw;
        uint16_t pid_integral_limit;
        uint8_t differential_sampling_coefficient;
        uint8_t load_step; // 每ms, 占空比最多改变若干个单位
        uint8_t position_step;
        uint16_t zero_offset; // 零位校准: 新零位 = 原始零位 + 该值, 顺时针旋转为正方向
        uint8_t run_mode;
        uint8_t angle_feedback_mode;
        uint16_t none2;
        uint8_t none3;
        uint8_t torque_enable;
        uint8_t none4;
        uint16_t goal_position;
        uint16_t goal_time;
        uint16_t goal_velocity;
        uint8_t eeprom_lock;
        uint16_t goal_circle;
        uint8_t relative_move_flag;
        uint32_t none5;
        uint16_t present_position;
        uint16_t present_velocity;
        uint16_t present_load;
        uint8_t present_voltage;
        uint8_t present_temperature;
        uint8_t reg_write_flag;
        uint8_t error;
        uint8_t running_flag;
        uint16_t present_goal_position;
        uint16_t present_current;
        uint16_t present_circle;
    } registers;
#pragma pack(pop) // 还原内存对齐
    
    static uint32_t offset_goal_position() { return offsetOf(&Register::goal_position); }
    static uint32_t sizeof_goal_position() { return sizeof(Register::goal_position); }
    
    static uint32_t offset_read_data() { return offsetOf(&Register::present_position); }
    static uint32_t sizeof_read_data() { return 17; }
    
    uint8_t *memory() {
        return (uint8_t *) &registers;
    }
    
    // 计算电机当前角度, unit: rad
    float present_position() {
        float pos = registers.present_position / 4096.0f * 2 * (float) M_PI;
        // 转换到[ -pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        return pos;
    }
    // 计算当前速度, unit: rad/s
    float present_velocity() {
        int vel;
        if (registers.present_velocity >= 32768) {
            vel = (int) 32767 - registers.present_velocity;
        } else {
            vel = registers.present_velocity;
        }
        float RPM = vel / 4096.0f;
        return RPM * 2 * (float) M_PI;
    }
    
    // 当前电机占空比, range: [-1024, 1023]
    int present_load_raw() {
        int load = registers.present_load;
        if (load >= 1024) {
            load = (int) 1023 - load;
        }
        return load;
    }
    // 当前电机占空比, range: [-1, 1]
    float present_load() {
        int load;
        if (registers.present_load >= 1024) {
            // [1024, 2047] 表示 [1, 1024]
            load = 1023 - (int) registers.present_load;
            return load / -1024.0f;
        } else {
            // [0, 1023] 表示 [-1023 ,0]
            load = registers.present_load;
            return load / -1023.0f;
        }
    }
    
    // 当前力矩, unit: unknown, 值>=100时可以认为抓到东西了
    int present_effort_raw() {
        int current = registers.present_current;
        if (current >= 32768) {
            current = (int) 32767 - current;
        }
        return current;
    }
    // 当前力矩
    float present_effort() {
        // 相对于最大力矩的百分比
        float effort_percent = present_effort_raw() / 450.0f;
        // 转换到kgcm
        float kgcm = effort_percent * 30.0f;
        // 转换到Nm
        float Nm = kgcm / 100.0f * 9.8f;
        return Nm;
    }
    
    // 设置电机目标位置, unit: rad, range: unlimited
    int goal_position(float _goal_position) {
        // 若圈数为反向, 则位置应取负值
        if (goal_circle() < 0) {
            _goal_position = -_goal_position;
        }
        // 转换到 [-pi, pi)
        _goal_position = std::remainder(_goal_position, 2 * (float) M_PI);
        // 转换到 [0, 2pi)
        if (_goal_position < 0)
            _goal_position += 2 * (float) M_PI;
        int pos = (uint16_t) (_goal_position / (2 * M_PI) * 4096);
        if (pos >= 4096)
            pos %= 4096;
        else if (pos < 0)
            pos = 0;
        registers.goal_position = (uint16_t) pos;
        return 0;
    }
    
    // 获取目标位置, unit: rad
    float goal_position() {
        float pos = registers.goal_position / 4096.0f * 2 * float(M_PI);
        // 转换到 [-pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        // 若圈数为反向, 则位置应取负值
        if (goal_circle() < 0) {
            pos = -pos;
        }
        return pos;
    }
    
    // 设置最大扭矩, range: [0, 1]
    int max_load(float _max_load) {
        // 限幅到 [0,1]
        _max_load = std::min(1.0f, _max_load);
        _max_load = std::max(0.0f, _max_load);
        // 转换为整型
        int _max_int = int(_max_load * 1024);
        // 限幅到 [0, 1023]
        _max_int = std::min(1023, _max_int);
        _max_int = std::max(0, _max_int);
        if (_max_int != registers.max_load) {
            registers.max_load = _max_int;
            return 0;
        } else {
            // 设定值与给定值相同
            return 1;
        }
    }
    
    // 获取当前目标位置反馈值, range: [ -pi, pi)
    float present_goal_position() {
        float pos = registers.present_goal_position / 4096.0f * 2 * float(M_PI);
        // 转换到[ -pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        return pos;
    }
    
    // 获取当前圈数
    int present_circle() {
        if (registers.present_circle >= 32768) {
            return (int) 32767 - 32768;
        } else {
            return registers.present_circle;
        }
    }
    // 获取当前目标圈数
    int goal_circle() {
        if (registers.goal_circle >= 32768) {
            return (int) 32767 - 32768;
        } else {
            return registers.goal_circle;
        }
    }
};

class OCServoRS485 : public OCServoDriver<OCServoRS485Device, 2> {
public:
    OCServoRS485(const char *name, DMABuffer_UART &rs485)
        : OCServoDriver(
        name, rs485,
        std::array<const char *, 2>{{"right_ee_joint", "left_ee_joint"}}) {
        this->ignore_busy_ = true;
    }
    
    void init() {
        OCServoOperation op;
        
        /* 修改ID */
        //        // 解锁ROM
        //        devices_[0].registers.eeprom_lock = 0;
        //        op.id = 0;
        //        op.instruction            = OCServoInstructionType::WRITE;
        //        op.reg_addr               = offsetOf(&Register::eeprom_lock);
        //        op.reg_length             = sizeof(OCServoRS485Device::Register::eeprom_lock);
        //        op.on_finish              = [this](const OCServoOperation& op) {
        //            printf("%s[%d]: eeprom unlocked\n", name, op.id);
        //        };
        //        add_operation(op);
        //        // 写入ID
        //        devices_[0].registers.id = 1;
        //        op.instruction            = OCServoInstructionType::WRITE;
        //        op.reg_addr               = offsetOf(&Register::id);
        //        op.reg_length             = sizeof(OCServoRS485Device::Register::id);
        //        op.on_finish              = [this](const OCServoOperation& op) {
        //            printf("%s[%d]: id overwrited 0 -> 1\n", name, op.id);
        //            while (true)
        //                ;
        //        };
        //        add_operation(op);
        
        // 初始位置
        devices_[0].goal_position(-(float)M_PI);
        devices_[1].goal_position(-(float)M_PI);
        
        // 零位
        devices_[0].registers.zero_offset = 2490; //1781;
        devices_[1].registers.zero_offset = 1799; //1781;
        
        // 初始化指令
        
        for (int i = 0; i < motor_count_; i++) {
            op.id = uint8_t(i + 1);
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
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::torque_enable);
            op.reg_length = sizeof(OCServoRS485Device::Register::torque_enable);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            // 写入零位
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::zero_offset);
            op.reg_length = sizeof(OCServoRS485Device::Register::zero_offset);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            // 设定并写入占空比步长
            devices_[i].registers.load_step = 255;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::load_step);
            op.reg_length = sizeof(OCServoRS485Device::Register::load_step);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            // 设定并写入默认最大占空比
            devices_[i].max_load(0.4);
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::max_load);
            op.reg_length = sizeof(OCServoRS485Device::Register::max_load);
            op.on_finish = decltype(op.on_finish)();
            add_operation(op);
            
            // 读取所有寄存器
            op.instruction = OCServoInstructionType::READ;
            op.reg_addr = 0;
            op.reg_length = sizeof(OCServoRS485Device::registers);
            op.on_finish = [this](const OCServoOperation &op) {
                // 读取到了当前圈数, 设设定目标圈数为当前圈数, 避免超出执行范围
                devices_[op.id - 1].registers.goal_circle = devices_[op.id - 1].registers.present_circle;
            };
            add_operation(op);
            
            // 写入目标圈数
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::goal_circle);
            op.reg_length = sizeof(OCServoRS485Device::Register::goal_circle);
            op.on_finish = [this](const OCServoOperation &op) {
                printf(
                    "%s[%d]: Connected, zero=%d, circle=%d, P=%d, I=%d(%d), D=%d(%d), dead=[%d,%d]\n",
                    name_, op.id,
                    devices_[op.id - 1].registers.zero_offset,
                    devices_[op.id - 1].registers.present_circle,
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
    
    // 设定指定电机的最大占空比 (异步)
    int set_max_load(int id, float max_load,
                     decltype(OCServoOperation::on_finish) on_finish = decltype(OCServoOperation::on_finish)()) {
        if (devices_[id - 1].max_load(max_load) == 0) {
            OCServoOperation op;
            op.id = (uint8_t) id;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::max_load);
            op.reg_length = sizeof(OCServoRS485Device::Register::max_load);
            //            [this](const OCServoOperation& op) -> bool {
            //                printf("%s[%d]: start set_max_load\n", name, op.id);
            //                return true;
            //            };
            op.on_finish = decltype(op.on_finish)();
            //            [this](const OCServoOperation& op) {
            //                printf("%s[%d]: set_max_load, success\n", name, op.id);
            //            };
            
            int ret = add_operation(op);
            if (ret >= 0) {
                return 0;
            } else {
                printf("%s[%d]: failed to set_max_load\n", name_, id);
                return ret;
            }
        } else {
            return 1;
        }
    }
};

//extern OCServoRS485 hand;
}
