#pragma once

#include <cassert>
#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <array>
#include <atomic>
#include <limits>
#include <stdexcept>

#include "stm32f4xx_hal.h"

#include "base_motor.hpp"
#include "base_motor_driver.hpp"
#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"
#include "main.h"
#include "multiple_circle_adapter.hpp"
#include "usart.h"
#include "utils.hpp"

#define CRC_ERROR 0x01
#define INSTRUCTION_ERROR 0x02
#define ADDRESS_ERROR 0x04

namespace hustac {

struct ZedaDevice {
    // 磁编码器一圈分辨率
    static const int motor_cycle_step_ = 16384;

#pragma pack(push) // 记录当前内存对齐
#pragma pack(1)    // 设定为1字节对齐
    // 内存区间总长度为0x53 (83字节)
    struct Register {
        uint8_t series;
        uint8_t model;
        uint8_t encoder;
        uint8_t brake;
        uint16_t motor_position_bias;
        uint8_t reduction_ratio;
        uint8_t pole_pairs_num;
        
        //        uint8_t *mechanical_version;
        //        uint8_t *hardware_version;
        //        uint8_t *software_version;
        
        int32_t power_off_position;
        
        uint8_t id;
        uint8_t baud_rate;
        int32_t position_bias;
        
        uint8_t position_control_period;
        uint8_t velocity_control_period;
        uint8_t current_control_period;
        
        uint16_t rom_position_p;
        uint16_t rom_position_i;
        uint16_t rom_position_d;
        uint16_t rom_velocity_p;
        uint16_t rom_velocity_i;
        uint16_t rom_velocity_d;
        uint16_t rom_current_p;
        uint16_t rom_current_i;
        uint16_t rom_current_d;
        
        uint16_t ram_position_p;
        uint16_t ram_position_i;
        uint16_t ram_position_d;
        uint16_t ram_velocity_p;
        uint16_t ram_velocity_i;
        uint16_t ram_velocity_d;
        uint16_t ram_current_p;
        uint16_t ram_current_i;
        uint16_t ram_current_d;
        
        uint8_t rounds;        // 圈数计数器
        uint8_t torque_enable; // 力矩使能 (0: 禁用电机)
        
        int32_t goal_position; // 目标角度
        int32_t goal_velocity; // 目标速度 (无效)
        int32_t goal_current;  // 目标电流 (无效)
        
        int32_t present_position; // 当前位置 = rounds * 16384 + 磁编码器值 + position_bias
        int32_t present_velocity; // 当前速度 (无效)
        int32_t present_current;  // 当前电流 (无效)
    } registers;
#pragma pack(pop) // 还原内存对齐
    
    static uint32_t offset_goal_position() { return offsetOf(&Register::goal_position); }
    static uint32_t sizeof_goal_position() { return sizeof(Register::goal_position); }
    
    static uint32_t offset_read_data() { return offsetOf(&Register::present_position); }
    static uint32_t sizeof_read_data() { return sizeof(&Register::present_position); }
    
    uint8_t *memory() {
        return (uint8_t *) &registers;
    }
    
    int32_t _max_position() {
        static_assert(motor_cycle_step_ % 2 == 0, "motor_cycle_step must divide by 2");
        return (motor_cycle_step_ / 2) * registers.reduction_ratio - 1;
    }
    
    int32_t _min_position() {
        static_assert(motor_cycle_step_ % 2 == 0, "motor_cycle_step must divide by 2");
        return -(motor_cycle_step_ / 2) * registers.reduction_ratio;
    }
    
    // 计算电机当前角度, 单位: rad, 有效范围-pi到+pi
    float present_position() {
        return float(registers.present_position) / motor_cycle_step_ / registers.reduction_ratio * 2 * float(M_PI);
    }
    
    // 设定电机目标角度, 单位: rad, 有效范围-pi到+pi
    int goal_position(float rad) {
        if (!std::isfinite(rad)) {
            return -1;
        }
        rad = std::min((float) M_PI, rad);
        rad = std::max((float) -M_PI, rad);
        int32_t pos = int32_t(rad / (2 * float(M_PI)) * registers.reduction_ratio * motor_cycle_step_);
        pos = std::min(_max_position(), pos);
        pos = std::max(_min_position(), pos);
        registers.goal_position = pos;
        return 0;
    }
    
    float goal_position() {
        return float(registers.goal_position) / motor_cycle_step_ / registers.reduction_ratio * 2 * float(M_PI);
    }
};

// 寄存器操作指令
enum class ZedaInstructionType : uint8_t {
    NONE = 0x00,
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    SYNC_WRITE_1 = 0x13,
    SYNC_WRITE_2 = 0x23,
};

struct ZedaOperation {
    uint8_t id = 0;
    ZedaInstructionType instruction = ZedaInstructionType::NONE;
    uint16_t reg_addr = 0;
    uint16_t reg_length = 0;
    std::function<void(const ZedaOperation &)> on_finish;
};

struct ZedaSyncOperation {
    uint8_t id_start = 0;
    uint8_t id_count = 0;
    ZedaInstructionType instruction = ZedaInstructionType::SYNC_WRITE_1;
    uint16_t reg_addr = 0;
    uint16_t reg_length = 0;
};

template<int motor_count>
class ZedaDriver : public BaseMotorDriver<ZedaOperation, ZedaSyncOperation> {
public:
    // 总线上的电机数量, 电机ID为1到N
    static const int motor_count_ = motor_count;
    
    // 请求数据包格式
    struct Request {
        uint8_t id;
        uint16_t length; // length长度为instruction+date+crc
        ZedaInstructionType instruction = ZedaInstructionType::NONE;
        uint8_t *data = NULL;
        uint16_t crc;
    };
    
    // 响应数据包格式
    struct Response {
        uint8_t id;
        uint16_t length; // length长度为instruction+error+date+crc
        ZedaInstructionType instruction = ZedaInstructionType::NONE;
        uint8_t error;
        uint8_t *data = NULL;
        uint16_t crc;
    };
    
    class MotorInterface : public BaseMotor {
    public:
        MotorInterface() {};
        MotorInterface(const char *name, ZedaDriver *pdriver, int id)
            : BaseMotor(name), pdriver_(pdriver), id_(id) {};
        
        int init() override { return 0; }
        int start() override { return pdriver_->enable(id_, true); }
        int stop() override { return pdriver_->enable(id_, false); }
        bool started() override { return pdriver_->motor_enabled_[id_ - 1]; }
        
        // 获得电机当前角度, 范围: 无限制
        float current_position() override {
            return multi_circle_.update(pdriver_->devices_[id_ - 1].present_position());
        }
        
        // 设定电机目标角度, 范围: logical_range
        int goal_position(float rad) override {
            if (!std::isfinite(rad)) {
                return -1;
            }
            // 限幅到logical_range
            rad = std::min(rad, logical_range_.second);
            rad = std::max(rad, logical_range_.first);
            goal_position_ = rad;
            // 计算当前位置到目标位置的偏差
            float delta = rad - current_position();
            // 限幅目标位置偏差, 避免反转
            float max_delta = (motor_range_.second - motor_range_.first) / 3;   // 120 deg
            delta = std::min(delta, max_delta);
            delta = std::max(delta, -max_delta);
            // 转换为电机位置
            float new_motor_pos = pdriver_->devices_[id_ - 1].present_position() + delta;
            new_motor_pos -= (motor_range_.second + motor_range_.first) / 2;
            new_motor_pos = std::remainder(new_motor_pos, motor_range_.second - motor_range_.first);
            new_motor_pos += (motor_range_.second + motor_range_.first) / 2;
            pdriver_->devices_[id_ - 1].goal_position(new_motor_pos);
            return 0;
        }
        
        float goal_position() override {
            return goal_position_;
        }
        
        std::pair<float, float> limit() override {
            return logical_range_;
        }
        
        int spin_once() override { return 0; };
    
    protected:
        ZedaDriver *pdriver_ = NULL;
        int id_ = -1;
        // 关节电机驱动程序的位置范围限制
        static constexpr std::pair<float, float> motor_range_ = {float(-M_PI), float(M_PI)};
        // 在增加了多圈适配器后的位置范围限制
        static constexpr std::pair<float, float> logical_range_ = {float(-2 * M_PI), float(2 * M_PI)};
        // 多圈适配器
        MultipleCircleAdapter<float> multi_circle_ = MultipleCircleAdapter<float>(motor_range_);
        float goal_position_ = std::numeric_limits<float>::signaling_NaN();
    };
    
    enum class RxState : uint8_t {
        ID = 0,
        INSTRUCTION = 1,
        LENGHTH_L = 2,
        LENGHTH_H = 3,
        Error = 4,
        DATA = 5,
        CRC_L = 6,
        CRC_H = 7,
    };

protected:
    bool ignore_protocol_ = false;
    
    Request current_req_;
    Response current_resp_;           // 当前接收的帧
    volatile RxState rx_state_ = RxState::ID; // 当前接收状态
    
    std::array<char *, motor_count_> joint_names_;
    std::array<std::pair<float, float>, motor_count_> joint_limits_;
    std::array<ZedaDevice, motor_count_> devices_;
    std::array<MotorInterface, motor_count_> motor_interface_;
    
    volatile uint8_t enabled_motor_id_start_ = 0;
    volatile uint8_t enabled_motor_id_count_ = 0;
    std::array<bool, motor_count_> motor_enabled_;
    
    std::array<float, motor_count> position_buffer_; // 读取位置数据的缓冲区

public:
    ZedaDriver(const char *name, DMABuffer_UART &rs485, const char *joint_prefix)
        : BaseMotorDriver(name, rs485) {
        this->ignore_timeout_ = false;
        this->ignore_busy_ = true;
        this->ignore_protocol_ = false;
        
        // 关节名称
        for (int id = 1; id <= motor_count; id++) {
            char tmp[20];
            snprintf(tmp, sizeof(tmp), "%s%d", joint_prefix, id);
            char *p = (char *) malloc(strlen(tmp) + 1);
            strcpy(p, tmp);
            joint_names_[id - 1] = p;
            motor_interface_[id - 1] = MotorInterface(joint_names_[id - 1], this, id);
            joint_limits_[id - 1] = motor_interface_[id - 1].limit();
        }
        
        // 初始化指令
        ZedaOperation op;
        
        for (int i = 0; i < motor_count; i++) {
            // 读取所有寄存器
            op.id = uint8_t(i + 1);
            op.instruction = ZedaInstructionType::READ;
            op.reg_addr = 0;
            op.reg_length = sizeof(typename ZedaDevice::Register);
            op.on_finish = [this](const ZedaOperation &op) {
                printf(
                    "%s[%d]: Connected, reduction=%d\n",
                    name_,
                    op.id,
                    devices_[op.id - 1].registers.reduction_ratio);
                if (op.id == motor_count_) {
                    on_init_finished();
                }
            };
            add_operation(op);
        }
    }
    
    ~ZedaDriver() {
        for (auto &p : joint_names_) {
            free(p);
            p = NULL;
        }
    }
    
    int motor_size() override { return motor_count_; }
    MotorInterface &motor(int id) override { return motor_interface_[id - 1]; }
    const decltype(position_buffer_) &position_buffer() { return position_buffer_; }
    const decltype(joint_names_) &joint_names() { return joint_names_; }
    const decltype(joint_limits_) &joint_limits() { return joint_limits_; }
    
    void print() {
        if (count_read_ <= 0) {
            // 尚未连接到电机
            printf("%s: Not connected Goal=[", name_);
            for (int i = 0; i < motor_count_; i++) {
                if (i > 0) {
                    printf(",");
                }
                printf("%.2f", devices_[i].goal_position() / float(M_PI) * 180.0f);
            }
            printf("]\n");
        } else {
            if (enabled_motor_id_start_ == 0 || enabled_motor_id_count_ == 0) {
                printf("%s: Enabled motor: None(%d,%d) Pos=[", name_, enabled_motor_id_start_, enabled_motor_id_count_);
            } else {
                printf("%s: Enabled motor: [%d,%d] Pos=[", name_, enabled_motor_id_start_,
                       enabled_motor_id_start_ + enabled_motor_id_count_ - 1);
            }
            for (int i = 0; i < motor_count_; i++) {
                if (i > 0) {
                    printf(",");
                }
                printf("%.2f", devices_[i].present_position() / float(M_PI) * 180.0f);
            }
            printf("] Goal=[");
            for (int i = 0; i < motor_count_; i++) {
                if (i > 0) {
                    printf(",");
                }
                printf("%.2f", devices_[i].goal_position() / float(M_PI) * 180.0f);
            }
            printf("]\n");
        }
    }
    
    int enable(int id, bool value) {
        if (id <= 0 || id > motor_count_) {
            return -1;
        }
        if (motor_enabled_[id - 1] == value) {
            return 0;
        }
        motor_enabled_[id - 1] = value;
        uint8_t id_front;
        uint8_t id_back;
        for (id_front = 1; id_front <= motor_count_; id_front++) {
            if (motor_enabled_[id_front - 1]) {
                break;
            }
        }
        for (id_back = id_front; id_back <= motor_count_; id_back++) {
            if (!motor_enabled_[id_back - 1]) {
                break;
            }
        }
        return enable_motor(id_front, id_back);
    }
    
    // 设定启用的电机范围 1-N则启动全部电机
    int enable_motor(uint8_t id_front, uint8_t id_back) {
        if (id_front >= 1 && id_front <= motor_count && id_back >= 1 && id_back <= motor_count && id_back >= id_front) {
            // ID合法
            if (!count_read_) {
                // 尚未连接到所有电机
                printf("%s: Failed to motor [%d, %d]! Not Connected\n", name_, int(id_front), int(id_back));
                return -1;
            }
        } else {
            // ID非法
            if (id_front != 0 || id_back != 0) {
                id_front = 0;
                id_back = 0;
                printf("%s: Enable motor of bad range [%d, %d]! Disable all motor\n", name_, int(id_front),
                       int(id_back));
            }
        }
        int id_start, id_count;
        if (id_back != 0 || id_front != 0) {
            // 有电机被启用
            if (id_back - id_front + 1 == motor_count) {
                // 全部电机启用
            } else {
                // 部分电机启用, 部分电机禁用, 需要单独发送禁用电机指令
                for (int id = enabled_motor_id_start_; id < enabled_motor_id_start_ + enabled_motor_id_count_; id++) {
                    if (id < id_front || id > id_back) {
                        // 找到本次被禁用的电机
                        devices_[id - 1].registers.torque_enable = 0;
                        ZedaOperation op;
                        op.id = (uint8_t)id;
                        op.instruction = ZedaInstructionType::WRITE;
                        op.reg_addr = (uint16_t)offsetOf(&ZedaDevice::Register::torque_enable);
                        op.reg_length = sizeof(ZedaDevice::Register::torque_enable);
                        if (add_operation(op) < 0) {
                            // 添加操作失败
                            printf("%s: Failed to add op: disable motor %d! Disable all motor\n", name_, int(id));
                            id_front = 0;
                            id_back = 0;
                            break;
                        } else {
                            // 成功添加操作
                            printf("%s: Manual disable motor %d!\n", name_, int(id));
                        }
                    }
                }
            }
            id_start = id_front;
            id_count = id_back - id_front + 1;
        } else {
            // 全部电机禁用
            id_start = 0;
            id_count = 0;
        }
        if (enabled_motor_id_start_ != id_start || enabled_motor_id_count_ != id_count) {
            enabled_motor_id_start_ = (uint8_t)id_start;
            enabled_motor_id_count_ = (uint8_t)id_count;
            printf("%s: Enabled motor: [%d, %d]!\n", name_, int(enabled_motor_id_start_),
                   int(enabled_motor_id_start_ + enabled_motor_id_count_ - 1));
        }
        return 0;
    }
    
    // 设定某一关节电机当前位置为零位(或其他指定位置)
    int calibrate(uint8_t id, float rad = 0) {
        if (id < 1 || id > motor_count) {
            printf("%s: Calibrate unknown motor %d\n", name_, int(id));
            return -1;
        }
        if (!std::isfinite(rad)) {
            printf("%s[%d][Calibrate]: bad angle %.2f\n", name_, int(id), rad);
            return -2;
        }
        rad = std::remainder(rad, 2 * float(M_PI));
        int real_pos = (int)std::floor(rad / (2 * float(M_PI)) * 16384);
        real_pos = std::min(real_pos, 8191);
        real_pos = std::max(real_pos, -8192);
        
        ZedaOperation op;
        op.id = id;
        op.instruction = ZedaInstructionType::READ;
        op.reg_addr = (uint16_t)offsetOf(&ZedaDevice::Register::rounds);
        op.reg_length = sizeof(ZedaDevice::Register::rounds);
        op.on_finish = [this, real_pos](const ZedaOperation &op) {
            int pos = devices_[op.id - 1].registers.present_position;
            int rounds = devices_[op.id - 1].registers.rounds;
            int bias = devices_[op.id - 1].registers.position_bias;
            int encoder = pos - bias - rounds * 16384;
            printf(
                "%s[%d][Calibrate]: pos=%d rounds=%d position_bias=%d encoder=%d real_pos=%d\n",
                name_, int(op.id), pos, rounds, bias, encoder, real_pos);
        };
        if (add_operation(op) < 0) {
            printf("%s[%d][Calibrate]: Failed to add op READ rounds\n", name_, int(id));
            return -1;
        }
        printf("%s[%d][Calibrate]: Start, keep stable!\n", name_, int(id));
        return 0;
    }
    
    // 解析一帧消息
    // 返回值: 是完整的一帧=0, 不是一帧=-1, 帧不完整=-2, CRC校验错误=-3
    int _parse_update(uint8_t ch) override {
        Response &resp = current_resp_;
        switch (rx_state_) {
        case RxState::ID:
            //            printf("%s: ID 0x%02X\n", name_, ch);
            if (ch == 0xFF) {
                // 跳过 0xFF
            } else if (ch != resp.id) {
                if (!this->ignore_protocol_)
                    printf("%s: bad ID=0x%02X (!=0x%02X)\n", name_, ch, resp.id);
                rx_state_ = RxState::ID;
            } else {
                resp.id = ch;
                rx_state_ = RxState::LENGHTH_L;
            }
            break;
        case RxState::LENGHTH_L:
            //            printf("%s: LENGHTH_L 0x%02X\n", name_, ch);
            if (ch != (resp.length & 0xFF)) {
                if (!this->ignore_protocol_)
                    printf("%s: bad LENGHTH_L=0x%02X (!=0x%02X)\n", name_, ch, resp.length & 0xFF);
                rx_state_ = RxState::ID;
            } else {
                resp.length = uint16_t((resp.length & 0xFF00) | ch);
                rx_state_ = RxState::LENGHTH_H;
            }
            break;
        case RxState::LENGHTH_H:
            //            printf("%s: LENGHTH_H 0x%02X\n", name_, ch);
            if (ch != (resp.length >> 8)) {
                if (!this->ignore_protocol_)
                    printf("%s: bad LENGHTH_H=0x%02X (!=0x%02X)\n", name_, ch, resp.length >> 8);
                rx_state_ = RxState::ID;
            } else {
                resp.length = uint16_t((resp.length & 0x00FF) | ((uint16_t) ch << 8));
                rx_state_ = RxState::INSTRUCTION;
            }
            break;
        case RxState::INSTRUCTION:
            //            printf("%s: INSTRUCTION 0x%02X\n", name_, ch);
            if (ch != (uint8_t) resp.instruction) {
                if (!this->ignore_protocol_)
                    printf("%s: bad INSTRUCTION=0x%02X (!=0x%02X)\n", name_, ch, (uint8_t) resp.instruction);
                rx_state_ = RxState::ID;
            } else {
                resp.instruction = (ZedaInstructionType) ch;
                rx_state_ = RxState::Error;
            }
            break;
        case RxState::Error:
            //            printf("%s: Error 0x%02X\n", name_, ch);
            if (ch) {
                if (!this->ignore_protocol_)
                    printf("%s: Unexpected error = 0x%02X\n", name_, ch);
            }
            resp.error = ch;
            if (resp.length - 4 > 0) {
                resp.data = devices_[resp.id - 1].memory() + current_op_.reg_addr;
                rx_state_ = RxState::DATA;
            } else {
                rx_state_ = RxState::CRC_L;
            }
            rx_frame_param_count_ = 0;
            break;
        case RxState::DATA:
            //            printf("%s: DATA 0x%02X\n", name_, ch);
            if (current_op_.instruction == ZedaInstructionType::READ) {
                resp.data[rx_frame_param_count_] = ch;
            }
            rx_frame_param_count_++;
            if (rx_frame_param_count_ >= resp.length - 4) { // 数据长度为包有效数据长度减4
                rx_state_ = RxState::CRC_L;
            }
            break;
        case RxState::CRC_L:
            //            printf("%s: CRC_L 0x%02X\n", name_, ch);
            resp.crc = ch;
            rx_state_ = RxState::CRC_H;
            break;
        case RxState::CRC_H:
            //            printf("%s: CRC_H 0x%02X\n", name_, ch);
            resp.crc |= uint16_t(ch) << 8;
            // todo: checksum
            rx_state_ = RxState::ID;
            return 0;
        }
        return 1;
    }
    
    void _generate_operation() override {
        while (pending_operations_.pop(current_op_) >= 0) {
            // 执行待执行的指定操作
            current_external_op_ = true;
            return;
            //            if (current_op_.on_start) {
            //                // 调用开始执行回调
            //                if (current_op_.on_start(current_op_)) {
            //                    // 回调返回true, 执行操作
            //                    current_op_.on_start = decltype(current_op_.on_start)();
            //                    return;
            //                }
            //                current_op_.on_start = decltype(current_op_.on_start)();
            //            } else {
            //                return;
            //            }
            //            printf("%s: external op\n", name_);
        }
        
        // 无待执行的操作, 执行默认操作
        current_external_op_ = false;
        
        // 读当前位置寄存器
        current_op_.id = current_operation_index_;
        current_op_.instruction = ZedaInstructionType::READ;
        current_op_.reg_addr = (uint16_t)ZedaDevice::offset_read_data();
        current_op_.reg_length = (uint16_t)ZedaDevice::sizeof_read_data();
        current_op_.on_finish = [this](const ZedaOperation &op) {
            if (op.id == motor_count) {
                // 将读回的数据拷贝到缓冲区
                for (int i = 0; i < motor_count; i++) {
                    position_buffer_[i] = motor_interface_[i].current_position();
                }
                count_read_++;
                if (read_pos_callback_) {
                    read_pos_callback_(*this);
                }
            }
            current_operation_index_++;
            if (current_operation_index_ > motor_count) {
                current_operation_index_ = 1;
            }
            //            printf("%s[%d]: load=% 5d current=% 6d\n", name_, op.id, devices_[op.id-1].registers.get_present_load_raw(), devices_[op.id-1].registers.get_present_effort_raw());
        };
    }
    
    // 执行一次"请求-响应"操作
    int _do_operation() override {
        ZedaOperation &op = current_op_;
        Request req;
        Response resp;
        
        
        int drop_count = this->uart_.readsome(NULL, 0xFFFF); // clear rx buffer
        if (drop_count > 0) {
            if (!this->ignore_protocol_)
                printf("%s: %d bytes droped\n", this->name_, drop_count);
        }
        rx_state_ = RxState::ID;
        
        // 帧头
        while (uart_.write((uint8_t *) "\xFF\xFF\xFF\xFF", 4, false) != 4);
        
        // ID
        req.id = op.id;
        resp.id = op.id;
        while (uart_.write(&(req.id), 1, false) != 1);
        
        // 帧长
        switch (op.instruction) {
        case ZedaInstructionType::PING:
            req.length = 0;
            resp.length = 0;
            break;
        case ZedaInstructionType::READ:
            req.length = 4;
            resp.length = op.reg_length;
            break;
        case ZedaInstructionType::WRITE:
            req.length = uint16_t(2) + op.reg_length;
            resp.length = 0;
            break;
        default:
            req.length = 0;
            resp.length = 0;
        }
        req.length += 3;
        resp.length += 4;
        while (uart_.write((uint8_t *) &(req.length), 2, false) != 2);
        
        // 指令
        req.instruction = op.instruction;
        resp.instruction = op.instruction;
        while (uart_.write((uint8_t *) &(req.instruction), 1, false) != 1);
        
        // 帧内容
        switch (op.instruction) {
        case ZedaInstructionType::PING:
            break;
        case ZedaInstructionType::READ:
            while (uart_.write((uint8_t *) &(op.reg_addr), 2, false) != 2);
            while (uart_.write((uint8_t *) &(op.reg_length), 2, false) != 2);
            break;
        case ZedaInstructionType::WRITE:
            while (uart_.write((uint8_t *) &(op.reg_addr), 2, false) != 2);
            while (uart_.write(&(devices_[op.id - 1].memory()[op.reg_addr]), op.reg_length, false) != op.reg_length);
            break;
        default:
            break;
        }
        req.data = NULL;
        resp.data = NULL;
        
        // CRC
        req.crc = 0x0000;
        while (uart_.write((uint8_t *) &(req.crc), 2, true) != 2);
        
        current_op_ = op;
        current_req_ = req;
        current_resp_ = resp;
        
        //        uart_.set_rx_callback(current_resp_.length + motor_count_, std::bind(&ZedaDriver::_on_rx_enough_bytes, this));
        return 0;
    }
    
    // 生成合适的同步写操作
    void _generate_sync_operation() override {
        if (enabled_motor_id_count_ > 0) {
            // 有部分电机被启动
            for (int id = enabled_motor_id_start_; id < enabled_motor_id_start_ + enabled_motor_id_count_; id++) {
                devices_[id - 1].registers.torque_enable = 1;
            }
            current_sync_op_.id_start = enabled_motor_id_start_;
            current_sync_op_.id_count = enabled_motor_id_count_;
            current_sync_op_.reg_addr = (uint16_t)offsetOf(&ZedaDevice::Register::goal_position);
            current_sync_op_.reg_length = sizeof(ZedaDevice::Register::goal_position);
            
            //            printf("%s[%d-%d]: write position\n", name_, current_sync_op_.id_start, current_sync_op_.id_start + current_sync_op_.id_count - 1);
        } else {
            // 所有电机被禁用
            for (int id = 1; id <= motor_count; id++) {
                devices_[id - 1].registers.torque_enable = 0;
            }
            current_sync_op_.id_start = 1;
            current_sync_op_.id_count = (uint8_t)motor_count;
            current_sync_op_.reg_addr = (uint16_t)offsetOf(&ZedaDevice::Register::torque_enable);
            current_sync_op_.reg_length = sizeof(ZedaDevice::Register::torque_enable);
            
            //            printf("%s[%d-%d]: write torque_enable\n", name_, current_sync_op_.id_start, current_sync_op_.id_start + current_sync_op_.id_count - 1);
        }
    }
    
    // 执行同步写操作
    int _do_sync_operation() override {
        ZedaSyncOperation &op = current_sync_op_;
        
        if (op.id_start < 1 || op.id_start > motor_count || op.id_start + op.id_count - 1 < 1 ||
            op.id_start + op.id_count - 1 > motor_count || op.instruction != ZedaInstructionType::SYNC_WRITE_1) {
            return -1;
        }
        
        Request req;
        
        int drop_count = this->uart_.readsome(NULL, 0xFFFF); // clear rx buffer
        if (drop_count > 0) {
            if (!this->ignore_protocol_)
                printf("%s: %d bytes droped\n", this->name_, drop_count);
        }
        rx_state_ = RxState::ID;
        
        // 帧头
        while (uart_.write((uint8_t *) "\xFF\xFF\xFF\xFF", 4, false) != 4);
        
        // ID
        req.id = 0xFE;
        while (uart_.write(&(req.id), 1, false) != 1);
        
        // 帧长
        switch (op.instruction) {
        case ZedaInstructionType::SYNC_WRITE_1:
            req.length = uint16_t(2 + 2 + 1 + op.id_count * op.reg_length);
            break;
        default:
            req.length = 0;
        }
        req.length += 3;
        while (uart_.write((uint8_t *) &(req.length), 2, false) != 2);
        
        // 指令
        req.instruction = op.instruction;
        while (uart_.write((uint8_t *) &(req.instruction), 1, false) != 1);
        
        // 帧内容
        switch (op.instruction) {
        case ZedaInstructionType::SYNC_WRITE_1:
            while (uart_.write((uint8_t *) &(op.reg_addr), 2, false) != 2);
            while (uart_.write((uint8_t *) &(op.reg_length), 2, false) != 2);
            while (uart_.write(&(op.id_start), 1, false) != 1);
            for (int id = op.id_start; id < op.id_start + op.id_count; id++) {
                while (uart_.write(&(devices_[id - 1].memory()[op.reg_addr]), op.reg_length, false) != op.reg_length);
            }
            break;
        default:
            break;
        }
        req.data = NULL;
        
        // CRC
        req.crc = 0x0000;
        while (uart_.write((uint8_t *) &(req.crc), 2, true) != 2);
        
        current_sync_op_ = op;
        current_req_ = req;
        current_resp_ = Response();
        return 0;
    }
};

using ArmMotorT = ZedaDriver<7>;
}
