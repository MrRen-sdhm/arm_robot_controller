#pragma once

#include <array>

#include "stm32f4xx_hal.h"

#include "main.h"
#include "usart.h"

#include "base_motor.hpp"
#include "base_motor_driver.hpp"
#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"
#include "utils.hpp"

namespace hustac {

struct OCServoDevice {

#pragma pack(push) // 记录当前内存对齐
#pragma pack(1)    // 设定为1字节对齐
    struct Register {
        // ...
        uint8_t torque_enable;
        // ...
    } registers;
#pragma pack(pop) // 还原内存对齐
    
    static uint32_t offset_torque_enable();
    static uint32_t sizeof_torque_enable();
    
    static uint32_t offset_goal_position();
    static uint32_t sizeof_goal_position();
    
    static uint32_t offset_read_data();
    static uint32_t sizeof_read_data();
    
    uint8_t *memory() {
        return (uint8_t *) &registers;
    }
    
    float present_position();
    float present_velocity();
    float present_acceleration();
    
    int goal_position(float);
    
    float goal_position();
    
    void torque_enable(bool);
    
    bool torque_enable();
};

// 寄存器操作指令
enum class OCServoInstructionType : uint8_t {
    NONE = 0x00,
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    RESET = 0x06,
    SYNC_WRITE = 0x83,
};

// 当前操作参数
struct OCServoOperation {
    uint8_t id = 0;
    OCServoInstructionType instruction = OCServoInstructionType::NONE;
    uint8_t reg_addr = 0;
    uint8_t reg_length = 0;
    std::function<void(const OCServoOperation &)> on_finish;
};
// 同步写操作参数
template<int motor_count>
struct OCServoSyncOperation {
    std::array<bool, motor_count> ids;
    OCServoInstructionType instruction = OCServoInstructionType::SYNC_WRITE;
    uint8_t reg_addr = 0;
    uint8_t reg_length = 0;
};

template<typename DeviceT, int motor_count>
class OCServoDriver : public BaseMotorDriver<OCServoOperation, OCServoSyncOperation<motor_count>> {
public:
    static const int motor_count_ = motor_count;
    
    // 抽象电机接口
    class MotorInterface : public BaseMotor {
    public:
        MotorInterface() {};
        MotorInterface(const char *name, OCServoDriver *pdriver, int id)
            : BaseMotor(name), pdriver_(pdriver), id_(id) {};
        
        int init() override { return 0; }
        int start() override { return pdriver_->enable(id_, true); }
        int stop() override { return pdriver_->enable(id_, false); }
        bool started() override { return pdriver_->motor_enabled_[id_ - 1]; }
        
        float goal_position() override {
            return pdriver_->devices_[id_ - 1].goal_position();
        }
        int goal_position(float pos) override {
            return pdriver_->devices_[id_ - 1].goal_position(pos);
        }
        
        float current_position() override {
            return pdriver_->devices_[id_ - 1].present_position();
        }
        float current_velocity() override {
            return pdriver_->devices_[id_ - 1].present_velocity();
        }
        float current_effort() override {
            return pdriver_->devices_[id_ - 1].present_effort();
        }
        
        int spin_once() override { return 0; };
    
    protected:
        OCServoDriver *pdriver_ = NULL;
        int id_ = 0;
    };
    
    // 数据包解析状态
    enum class RxState : uint8_t {
        ID = 0,
        LENGTH = 1,
        DATA = 2,
        WORK_STATE = 3,
        CHECK_SUM = 4,
    };
    
    // 请求数据包格式
    struct Request {
        uint8_t id;
        uint8_t length; // id、指令、数据的字节数之和(数据为 地址+要读取的字节数)
        OCServoInstructionType instruction = OCServoInstructionType::NONE;
        uint8_t *data = NULL;
        uint8_t check_sum;
    };
    
    // 响应数据包格式
    struct Response {
        uint8_t id;
        uint8_t length; // id、指令、数据的字节数之和(数据为 返回的各寄存器参数)
        uint8_t error;
        uint8_t *data = NULL;
        uint8_t check_sum;
        
        uint8_t check() {
            uint8_t checksum = 0;
            checksum += id;
            checksum += length;
            checksum += error;
            for (int i = 0; i < length - 2; i++) {
                checksum += data[i];
            }
//            checksum ^= (uint8_t)0x02;
            uint8_t badbit = ~checksum ^check_sum;
            return badbit;
        }
    };

protected:
    bool ignore_protocol_ = false;
    
    Request current_req_;
    Response current_resp_;           // 当前接收的帧
    volatile RxState rx_state_ = RxState::ID; // 当前接收状态
    
    std::array<const char *, motor_count_> joint_names_;
    std::array<DeviceT, motor_count_> devices_;
    std::array<MotorInterface, motor_count_> motor_interface_;
    
    std::array<bool, motor_count_> motor_enabled_; // 电机使能状态
    
    std::array<float, motor_count_> position_buffer_; // 读取位置数据的缓冲区
    std::array<float, motor_count_> velocity_buffer_; // 读取速度数据的缓冲区
    std::array<float, motor_count_> effort_buffer_;   // 读取力矩数据的缓冲区

public:
    OCServoDriver(const char *name, DMABuffer_UART &rs485, decltype(joint_names_) joint_names)
        : BaseMotorDriver<OCServoOperation, OCServoSyncOperation<motor_count>>(name, rs485), joint_names_(joint_names) {
        for (int i = 0; i < motor_count_; i++) {
            motor_interface_[i] = MotorInterface(joint_names_[i], this, i + 1);
        }
    }
    
    // 初始化指令, 应由子类实现, 至少应读取所有寄存器
    void init();
    
    int motor_size() override { return motor_count_; }
    MotorInterface &motor(int id) override { return motor_interface_[id - 1]; }
    const decltype(joint_names_) &joint_names() { return joint_names_; }
    const decltype(position_buffer_) &position_buffer() { return position_buffer_; }
    const decltype(velocity_buffer_) &velocity_buffer() { return velocity_buffer_; }
    const decltype(effort_buffer_) &effort_buffer() { return effort_buffer_; }
    
    int enable(uint8_t id, bool state) {
        int ret;
        if (1 <= id && id <= motor_count_) {
            InterruptLock lock;
            std::lock_guard<InterruptLock> lock_guard(lock);
            if (motor_enabled_[id - 1] && !state) {
                // disable
                OCServoOperation op;
                devices_[id - 1].registers.torque_enable = 0;
                op.id = id;
                op.instruction = OCServoInstructionType::WRITE;
                op.reg_addr = (uint16_t) offsetOf(&DeviceT::Register::torque_enable);
                op.reg_length = sizeof(DeviceT::Register::torque_enable);
                op.on_finish = [this](const OCServoOperation &op) {
                    printf("%s[%d]: torgue disabled\n", this->name_, op.id);
                };
                ret = this->add_operation(op);
                if (ret >= 0) {
                    motor_enabled_[id - 1] = false;
                    printf("%s: disable motor %d\n", this->name_, id);
                } else {
                    printf("%s: failed to disable motor %d\n", this->name_, id);
                }
            } else if (!motor_enabled_[id - 1] && state) {
                // enable
                devices_[id - 1].registers.torque_enable = 1;
                motor_enabled_[id - 1] = true;
                printf("%s: enable motor %d\n", this->name_, id);
                ret = 0;
            } else {
                ret = 0;
            }
        } else {
            ret = -1;
        }
        return ret;
    }
    
    // 解析一帧消息
    // 返回值: 是完整的一帧=0, 不是一帧=-1
    int _parse_update(uint8_t ch) override {
        //        printf("[%02X]", ch);
        Response &resp = current_resp_;
        switch (rx_state_) {
        case RxState::ID:
            if (ch == 0xFF) {
                // 跳过 0xFF
            } else if (ch != resp.id) {
                rx_state_ = RxState::ID;
                if (!this->ignore_protocol_)
                    printf("%s: bad id %02X (!= %02X)\n", this->name_, ch, resp.id);
            } else {
                resp.id = ch;
                rx_state_ = RxState::LENGTH;
            }
            break;
        case RxState::LENGTH:
            if (ch != resp.length) {
                rx_state_ = RxState::ID;
                if (!this->ignore_protocol_)
                    printf("%s: bad length %d (!= %d)\n", this->name_, ch, resp.length);
            } else {
                rx_state_ = RxState::WORK_STATE;
            }
            break;
        case RxState::WORK_STATE:
            resp.error = ch;
            if (resp.length > 2) {
                rx_state_ = RxState::DATA;
                resp.data = devices_[resp.id - 1].memory() + this->current_op_.reg_addr;
                this->rx_frame_param_count_ = 0;
            } else {
                rx_state_ = RxState::CHECK_SUM;
            }
            break;
        case RxState::DATA:
            if (this->current_op_.instruction == OCServoInstructionType::READ) {
                resp.data[this->rx_frame_param_count_] = ch;
            }
            this->rx_frame_param_count_++;
            if (this->rx_frame_param_count_ >= resp.length - 2) { // 数据长度为包有效数据长度减2
                this->rx_frame_param_count_ = 0;
                rx_state_ = RxState::CHECK_SUM;
            }
            break;
        case RxState::CHECK_SUM:
            resp.check_sum = ch;
            rx_state_ = RxState::ID;
            uint8_t check = resp.check();
            if (check == 0) {
//                printf("%s: Good frame\n", this->name_);
                return 0;
            } else if (!this->ignore_protocol_) {
                printf("%s: bad checksum %02X\n", this->name_, check);
                return -1;
            }
        }
        return 1;
    }
    
    void _generate_operation() override {
        while (this->pending_operations_.pop(this->current_op_) >= 0) {
            // 执行待执行的指定操作
            this->current_external_op_ = true;
            return;
        }
        
        // 无待执行的操作, 执行默认操作
        this->current_external_op_ = false;
        
        // 读当前位置及之后的所有寄存器
        this->current_op_.id = this->current_operation_index_;
        this->current_op_.instruction = OCServoInstructionType::READ;
        this->current_op_.reg_addr = DeviceT::offset_read_data();
        this->current_op_.reg_length = DeviceT::sizeof_read_data();
        this->current_op_.on_finish = [this](const OCServoOperation &op) {
            if (op.id == motor_count_) {
                // 将读回的数据拷贝到缓冲区
                for (int i = 0; i < motor_count_; i++) {
                    position_buffer_[i] = devices_[i].present_position();
                    velocity_buffer_[i] = devices_[i].present_velocity();
                    effort_buffer_[i] = devices_[i].present_effort();
                }
                //                printf("%s: [1]=%d [2]=%d\n", name, motor[0].registers.present_position, motor[1].registers.present_position);
                //                printf("%s: [1]=%.2f [2]=%.2f\n", name, motor[0].registers.get_present_load(), motor[1].registers.get_present_load());
                this->count_read_++;
                if (this->read_pos_callback_) {
                    this->read_pos_callback_(*this);
                }
            }
            this->current_operation_index_++;
            if (this->current_operation_index_ > motor_count_) {
                this->current_operation_index_ = 1;
            }
            //            printf("%s[%d]: load=% 5d current=% 6d\n", name, op.id, motor[op.id-1].registers.get_present_load_raw(), motor[op.id-1].registers.get_present_effort_raw());
        };
        //            printf("%s: default op\n", name);
    }
    
    // 执行一次"请求-响应"操作, 读或写(首地址+长度)
    int _do_operation() override {
        OCServoOperation &op = this->current_op_;
        Request req;
        Response resp;
        
        int drop_count = this->uart_.readsome(NULL, 0xFFFF); // clear rx buffer
        if (drop_count > 0) {
            if (!this->ignore_protocol_)
                printf("%s: %d bytes droped\n", this->name_, drop_count);
        }
        rx_state_ = RxState::ID;
        
        // 帧头
        while (this->uart_.write((uint8_t *) "\xFF\xFF", 2, false) != 2);
        
        // ID
        req.id = op.id;
        resp.id = op.id;
        while (this->uart_.write(&(req.id), 1, false) != 1);
        
        // 帧长
        switch (op.instruction) {
        case OCServoInstructionType::PING:
            req.length = 2;
            resp.length = 0;
            break;
        case OCServoInstructionType::READ:
            req.length = 4;
            resp.length = op.reg_length;
            break;
        case OCServoInstructionType::WRITE:
            req.length = op.reg_length + (uint8_t) 3;
            resp.length = 0;
            break;
        case OCServoInstructionType::REG_WRITE:
            req.length = op.reg_length + (uint8_t) 2;
            resp.length = 0;
            break;
        case OCServoInstructionType::ACTION:
            req.length = 2;
            resp.length = 0;
            break;
        case OCServoInstructionType::RESET:
            req.length = 2;
            resp.length = 0;
            break;
        default:
            req.length = 0;
            resp.length = 0;
        }
        resp.length += 2;
        while (this->uart_.write((uint8_t *) &(req.length), 1, false) != 1);
        
        // 指令
        req.instruction = op.instruction;
        while (this->uart_.write((uint8_t *) &(req.instruction), 1, false) != 1);
        
        // 帧内容
        switch (op.instruction) {
        case OCServoInstructionType::PING:
            break;
        case OCServoInstructionType::READ:
            while (this->uart_.write((uint8_t *) &(op.reg_addr), 1, false) != 1);
            while (this->uart_.write((uint8_t *) &(op.reg_length), 1, false) != 1);
            break;
        case OCServoInstructionType::WRITE:
            while (this->uart_.write((uint8_t *) &(op.reg_addr), 1, false) != 1);
            while (this->uart_.write(&(devices_[op.id - 1].memory()[op.reg_addr]), op.reg_length, false) !=
                   op.reg_length);
            break;
        case OCServoInstructionType::REG_WRITE:
            while (this->uart_.write((uint8_t *) &(op.reg_addr), 1, false) != 1);
            while (this->uart_.write(&(devices_[op.id - 1].memory()[op.reg_addr]), op.reg_length, false) !=
                   op.reg_length);
            break;
        case OCServoInstructionType::ACTION:
            break;
        case OCServoInstructionType::RESET:
            break;
        default:
            break;
        }
        req.data = NULL;
        resp.data = NULL;
        
        // 校验和
        req.check_sum = 0;
        switch (op.instruction) {
        case OCServoInstructionType::WRITE:
        case OCServoInstructionType::REG_WRITE:
            req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr;
            for (int i = 0; i < op.reg_length; i++) {
                req.check_sum += devices_[op.id - 1].memory()[op.reg_addr + i];
            }
            break;
        case OCServoInstructionType::READ:
            req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr + op.reg_length;
            break;
        default:
            break;
        }
        req.check_sum = ~req.check_sum;
        while (this->uart_.write(&(req.check_sum), 1, true) != 1);
        
        this->current_op_ = op;
        current_req_ = req;
        current_resp_ = resp;
        
        return 0;
    }
    
    // 生成合适的同步写操作
    void _generate_sync_operation() override {
        bool all_motor_disabled = true;
        for (const bool &enabled : motor_enabled_) {
            if (enabled) {
                all_motor_disabled = false;
                break;
            }
        }
        
        if (!all_motor_disabled) {
            // 有部分电机被启动
            for (int id = 1; id <= motor_count_; id++) {
                devices_[id - 1].registers.torque_enable = motor_enabled_[id - 1];
            }
            this->current_sync_op_.ids = motor_enabled_;
            this->current_sync_op_.reg_addr = DeviceT::offset_goal_position();
            this->current_sync_op_.reg_length = DeviceT::sizeof_goal_position();
            
            //            printf("%s[%d-%d]: write position\n", name, current_sync_op.id_start, current_sync_op.id_start + current_sync_op.id_count - 1);
        } else {
            // 所有电机被禁用
            for (int id = 1; id <= motor_count_; id++) {
                devices_[id - 1].registers.torque_enable = 0;
                this->current_sync_op_.ids[id - 1] = true;
            }
            this->current_sync_op_.reg_addr = (uint16_t) offsetOf(&DeviceT::Register::torque_enable);
            this->current_sync_op_.reg_length = sizeof(DeviceT::Register::torque_enable);
            
            //            printf("%s[%d-%d]: write torque_enable\n", name, current_sync_op.id_start, current_sync_op.id_start + current_sync_op.id_count - 1);
        }
    }
    
    // 执行同步写操作
    int _do_sync_operation() override {
        OCServoSyncOperation<motor_count_> &op = this->current_sync_op_;
        
        int id_count = 0;
        for (const bool &enabled : motor_enabled_) {
            if (enabled) {
                id_count++;
            }
        }
        
        Request req;
        
        int drop_count = this->uart_.readsome(NULL, 0xFFFF); // clear rx buffer
        if (drop_count > 0) {
            if (!this->ignore_protocol_)
                printf("%s: %d bytes droped\n", this->name_, drop_count);
        }
        rx_state_ = RxState::ID;
        
        // 帧头
        while (this->uart_.write((uint8_t *) "\xFF\xFF", 2, false) != 2);
        
        // ID
        req.id = 0xFE;
        while (this->uart_.write(&(req.id), 1, false) != 1);
        
        // 帧长
        switch (op.instruction) {
        case OCServoInstructionType::SYNC_WRITE:
            req.length = uint16_t(4 + id_count * (op.reg_length + 1)); // 发送的数据中包含id
            break;
        default:
            req.length = 0;
        }
        while (this->uart_.write((uint8_t *) &(req.length), 1, false) != 1);
        
        // 指令
        req.instruction = op.instruction;
        while (this->uart_.write((uint8_t *) &(req.instruction), 1, false) != 1);
        
        // 帧内容
        switch (op.instruction) {
        case OCServoInstructionType::SYNC_WRITE:
            while (this->uart_.write((uint8_t *) &(op.reg_addr), 1, false) != 1);
            while (this->uart_.write((uint8_t *) &(op.reg_length), 1, false) != 1);
            for (uint8_t id = 1; id <= motor_count_; id++) {
                if (op.ids[id - 1]) {
                    while (this->uart_.write(&id, 1, false) != 1);
                    while (this->uart_.write(&(devices_[id - 1].memory()[op.reg_addr]), op.reg_length, false) !=
                           op.reg_length);
                }
            }
            break;
        default:
            break;
        }
        req.data = NULL;
        
        // 校验和
        req.check_sum = 0;
        // 各舵机数据和
        for (int id = 1; id <= motor_count_; id++) {
            if (op.ids[id - 1]) {
                req.check_sum += id;
                for (int i = 0; i < op.reg_length; i++) {
                    req.check_sum += (&devices_[id - 1].memory()[op.reg_addr])[i];
                }
            }
        }
        req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr + op.reg_length;
        req.check_sum = ~req.check_sum;
        while (this->uart_.write(&(req.check_sum), 1, true) != 1);
        
        this->current_sync_op_ = op;
        current_req_ = req;
        current_resp_ = Response();
        return 0;
    }
};
}