#pragma once

#include <cmath>
#include <array>
#include <algorithm>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

#include "high_resolution_clock.h"
#include "dmabuffer_uart.hpp"

namespace hustac {

// RoboMaster C620 Brushless DC Motor
class C620Motor {
public:
#pragma pack(push)          // 记录当前内存对齐
#pragma pack(1)             // 设定为1字节对齐
    
    struct FeedbackFrame {
        uint8_t position_h;
        uint8_t position_l;
        uint8_t velocity_h;
        uint8_t velocity_l;
        uint8_t current_h;
        uint8_t current_l;
        uint8_t temperature;
        uint8_t none;
        
        uint16_t position_raw() const {
            return (uint16_t(position_h) << 8) | position_l;
        }
        int16_t velocity_raw() const {
            return (uint16_t(velocity_h) << 8) | velocity_l;
        }
        int16_t current_raw() const {
            return (uint16_t(current_h) << 8) | current_l;
        }
        // uint: rad, range: [0, 2*pi)
        float position() const {
            float pos = position_raw() / 8192.0f * 2 * float(M_PI);
            if (pos >= M_PI) {
                pos -= 2 * M_PI;
            }
            return pos;
        }
        // unit: rad/s
        float velocity() const {
            return velocity_raw() / 60.0f * 2 * float(M_PI);
        }
        // unit: A
        float current() const {
            return current_raw() / 16384.0f * 20;
        }
        // uint: Nm
        float torque() const {
            // about 5Nm at 14A
            return current() / 14.0f * 5;
        }
        void print(const char* prefix = "") const {
            printf(
                "%spos=%.2f deg, vel=%.2f deg/s, current=%.2f A (%.2f Nm)\n",
                prefix,
                position() / float(M_PI) * 180,
                velocity() / float(M_PI) * 180,
                current(),
                torque()
            );
        }
    };
    
    struct ControlFrame {
        uint8_t id_1_current_h;
        uint8_t id_1_current_l;
        uint8_t id_2_current_h;
        uint8_t id_2_current_l;
        uint8_t id_3_current_h;
        uint8_t id_3_current_l;
        uint8_t id_4_current_h;
        uint8_t id_4_current_l;
        
        int16_t get_current_raw(int id) {
            uint8_t* p = (uint8_t*) this + (id - 1) * 2;
            return int16_t(p[0]) << 8 | p[1];
        }
        int16_t set_current_raw(int id, int16_t raw) {
            uint8_t* p = (uint8_t*) this + (id - 1) * 2;
            // limit to [-16384, 16384]
            raw = std::min((int16_t) 16384, raw);
            raw = std::max((int16_t) -16384, raw);
            p[0] = raw >> 8;
            p[1] = raw;
            return raw;
        }
        float get_current(int id) {
            return get_current_raw(id) / 16384.0f * 20;
        }
        float set_current(int id, float c) {
            // limit to [-20, 20]
            c = std::min(20.0f, c);
            c = std::max(-20.0f, c);
            set_current_raw(id, int16_t(c / 20 * 16384));
            return get_current(id);
        }
        float get_effort(int id) {
            return get_current(id) / 14.0f * 5;
        }
        float set_effort(int id, float e) {
            set_current(id, e * 14.0f / 5);
            return get_effort(id);
        }
    };

#pragma pack(pop)   // 还原内存对齐
    
    enum class FeedbackFlag : uint8_t {
        NONE_RECVED = 0x00,
        ID_1_RECVED = 0x01,
        ID_2_RECVED = 0x02,
        ID_3_RECVED = 0x04,
        ID_4_RECVED = 0x08,
        ALL_RECVED = 0x0F,
    };
    
    enum class Motor : uint8_t {
        LEFT_FRONT = 0x00,
        LEFT_BACK = 0x01,
        RIGHT_BACK = 0x02,
        RIGHT_FRONT = 0x03,
    };
    
    struct Wheel {
        const float reduction = 3591 / 187.0f;
        // torque of current: 0.3 Nm/A
        const float max_effort = 20 * 0.3f * reduction; // 115 Nm
        int count_circle = 0;
        float last_inner_position = 0;
        uint64_t last_recv_ok = 0;
        
        float position;
        float velocity;
        float current;
        float effort;
        float desired_effort = 0;
        
        void update(const FeedbackFrame& fb, bool reverse = false) {
            float inner_position = reverse ? -fb.position() : fb.position();
            if (inner_position - last_inner_position > M_PI) {
                count_circle--;
            } else if (inner_position - last_inner_position < -M_PI) {
                count_circle++;
            }
            last_inner_position = inner_position;
            
            position = std::remainder(count_circle * (2 * M_PI / reduction) + inner_position / reduction, 2 * M_PI);
            velocity = (reverse ? -fb.velocity() : fb.velocity()) / reduction;
            current = reverse ? -fb.current() : fb.current();
            effort = current * 0.3f * reduction;
        }
        
        float get_desired_current() {
            return desired_effort / reduction / 0.3f;
        }
    };
    
    const char* name_;
    CAN_HandleTypeDef* hcan_;
    SoftTimerMS<1000> timer_print_ = {0};
    
    // 电机的id号, 左前, 左后, 右后, 右前
    std::array<uint8_t, 4> motor_ids_ = {2, 3, 4, 1};
    
    std::array<FeedbackFrame, 4> fb_frames_ = {};
    ControlFrame ctrl_frame_ = {};
    
    bool connected_ = false;
    uint8_t fb_flag_ = (uint8_t) FeedbackFlag::NONE_RECVED;
    
    uint64_t last_all_recved_ = 0;
    float interval_ = 0;
    
    std::array<Wheel, 4> wheels_;
    
    C620Motor(const char* name, CAN_HandleTypeDef* hcan)
        : name_(name), hcan_(hcan) {
    }
    
    int init() {
        connected_ = false;
        
        CAN_FilterTypeDef filt_cfg;
        filt_cfg.FilterBank = 0;   // 配置Bank0的过滤器
        filt_cfg.FilterActivation = ENABLE; // 启用该过滤器
        filt_cfg.FilterFIFOAssignment = CAN_FILTER_FIFO0;   // 该过滤器输出到FIFO0
        filt_cfg.FilterMode = CAN_FILTERMODE_IDMASK;    // 使用掩码匹配模式, 若 ID & FilterMaskId == FilterId & FilterMaskId 则通过过滤器
        filt_cfg.FilterScale = CAN_FILTERSCALE_32BIT;   // 使用单个32位掩码
        filt_cfg.FilterMaskIdHigh = 0x0000; // 掩码 高16位, 全0表示全部通过
        filt_cfg.FilterMaskIdLow = 0x0000;  // 掩码 低16位
        filt_cfg.FilterIdHigh = 0x0000; // 校验码 高16位
        filt_cfg.FilterIdLow = 0x0000;  // 校验码 低16位
        filt_cfg.SlaveStartFilterBank = 14;  // CAN1 使用 Bank0 ~ Bank13 的过滤器, CAN2 使用 Bank14 ~ Bank27 的过滤器
        if (HAL_CAN_ConfigFilter(hcan_, &filt_cfg) != HAL_OK) {
            printf("%s: HAL_CAN_ConfigFilter() failed!!!\n", name_);
            return -1;
        }
//        if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
//                                                CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR | CAN_IT_RX_FIFO0_OVERRUN |
//                                                CAN_IT_RX_FIFO1_OVERRUN) != HAL_OK) {
//            printf("%s: HAL_CAN_ActivateNotification(ERROR) failed!!!\n", name_);
//            return -1;
//        }
//        if (HAL_CAN_ActivateNotification(hcan_,
//                                         CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL |
//                                         CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL) != HAL_OK) {
//            printf("%s: HAL_CAN_ActivateNotification(RX) failed!!!\n", name_);
//            return -1;
//        }
        if (HAL_CAN_Start(hcan_) != HAL_OK) {
            printf("%s: HAL_CAN_Start() failed!!!\n", name_);
            return -1;
        }
        printf("%s: Started\n", name_);
        timer_print_.reset();
        return 0;
    }
    
    bool connected() {
        return connected_;
    }
    
    int read() {
        int ret = -1;
        while (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0) > 0) {
            CAN_RxHeaderTypeDef header;
            FeedbackFrame fb;
            if (HAL_CAN_GetRxMessage(hcan_, CAN_FILTER_FIFO0, &header, (uint8_t*) &fb) != HAL_OK) {
                printf("%s: HAL_CAN_GetRxMessage() failed\n", name_);
                break;
            }
            
            uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
//            printf("%s: recved frame from id 0x%X\n", name_, header.StdId);
            switch (header.StdId) {
            case 0x201:
                fb_flag_ |= (uint8_t) FeedbackFlag::ID_1_RECVED;
                fb_frames_[0] = fb;
                wheels_[0].last_recv_ok = now;
                break;
            case 0x202:
                fb_flag_ |= (uint8_t) FeedbackFlag::ID_2_RECVED;
                fb_frames_[1] = fb;
                wheels_[1].last_recv_ok = now;
                break;
            case 0x203:
                fb_flag_ |= (uint8_t) FeedbackFlag::ID_3_RECVED;
                fb_frames_[2] = fb;
                wheels_[2].last_recv_ok = now;
                break;
            case 0x204:
                fb_flag_ |= (uint8_t) FeedbackFlag::ID_4_RECVED;
                fb_frames_[3] = fb;
                wheels_[3].last_recv_ok = now;
                break;
            default:
                // unknown id
                printf("%s: recved frame from unknown id 0x%X\n", name_, header.StdId);
                break;
            }
            if (fb_flag_ == (uint8_t) FeedbackFlag::ALL_RECVED) {
                if (last_all_recved_) {
                    interval_ = (now - last_all_recved_) / 1e9f;
                }
                last_all_recved_ = now;
                if (!connected_) {
                    connected_ = true;
                    printf("%s: Connected to motor\n", name_);
                }
                
                wheels_[0].update(fb_frames_[motor_ids_[0] - 1], false);
                wheels_[1].update(fb_frames_[motor_ids_[1] - 1], false);
                wheels_[2].update(fb_frames_[motor_ids_[2] - 1], true);
                wheels_[3].update(fb_frames_[motor_ids_[3] - 1], true);
                
                //                printf("%s: feedback recved\n", name_);
                fb_flag_ = (uint8_t) FeedbackFlag::NONE_RECVED;
                ret = 0;
                break;
            }
        }
        
        timer_print_.is_timeout([this](uint32_t count) -> bool {
            int can_state = HAL_CAN_GetState(hcan_);
            if (can_state != HAL_CAN_STATE_LISTENING) {
                printf("%s: CAN bad status %d\n", name_, can_state);
                return true;
            }
            bool printed = false;
            uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
            for (int i = 0; i < 4; i++) {
                if (now - wheels_[i].last_recv_ok >= 10000000) {
                    if (!printed)
                        printf("%s: ID", name_);
                    printf(" %d", i + 1);
                    printed = true;
                }
            }
            if (printed) {
                puts(" lost");
            }
            return printed;
        });
        return ret;
    }
    
    Wheel
    
    &
    operator[](Motor
               motor) {
        return (*this)[(uint8_t) motor];
    }
    Wheel& operator[](uint8_t motor) {
        return wheels_[motor];
    }
    
    int write() {
        ctrl_frame_.set_current(motor_ids_[0], wheels_[0].get_desired_current());
        ctrl_frame_.set_current(motor_ids_[1], wheels_[1].get_desired_current());
        ctrl_frame_.set_current(motor_ids_[2], -wheels_[2].get_desired_current());
        ctrl_frame_.set_current(motor_ids_[3], -wheels_[3].get_desired_current());
        
        CAN_TxHeaderTypeDef header;
        uint32_t tx_mailbox;
        header.StdId = 0x200;
        header.IDE = CAN_ID_STD;    // 不使用扩展标识符
        header.RTR = CAN_RTR_DATA;  // 消息类型为数据帧
        header.DLC = 0x08;          // 一帧8字节
        header.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(hcan_, &header, (uint8_t*) &ctrl_frame_, &tx_mailbox) == HAL_OK) {
//            printf("%s: ctrl sent\n", name_);
            return 0;
        } else {
            printf("%s: failed to send ctrl!\n", name_);
            return -1;
        }
    }
};

extern C620Motor macnum_motor;
    
}
