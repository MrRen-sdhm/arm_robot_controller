#pragma once

#include <algorithm>
#include <array>

#include "stm32f4xx_hal.h"

#include "dmabuffer_uart.hpp"
#include "gpio.hpp"
#include "utils.hpp"

namespace hustac {

class StepMotor {
public:
    static const uint32_t tim_freq = 1000000;
    static const int PPR           = 6400; // Number of pulses per rotation
    static const int AGTN          = 16; // Number of active gear teeth
    static const int DGTN          = 24; // Number of deactive gear teeth
    static const int PH            = 10; // Lead of the screw
    static float pulse_to_meter(int32_t pulse) {
        return (float)pulse / PPR * ((float)AGTN / DGTN * PH) / 1000;
    }
    static int32_t meter_to_pulse(float meter) {
        return (int32_t)(meter * 1000 / ((float)AGTN / DGTN * PH) * PPR);
    }

    const char* name_;
    TIM_HandleTypeDef* htim; // handle of lift TIM
    uint32_t Channel; // channle of lift TIM
    GPIOOutput& pin_disable_;
    GPIOOutput& pin_unlock_;
    GPIOOutput& pin_dir_down_;
    GPIOOutput& pin_pulse_;

    int32_t pulse_count_         = 0; // 期望的位置, 脉冲发送数
    volatile int32_t pulse_sent_ = 0; // 已发送的脉冲数
    int32_t pulse_period_        = 0; // 期望的脉冲发送周期, 单位: us
    volatile bool moving_        = false;
    bool pulse_high_             = false;
    std::function<void(int32_t pulse_sent)> stop_callback_;

    static const int delay_lock_ = 1000;
    uint32_t last_moving_        = 0;

    StepMotor(const char* name,
        TIM_HandleTypeDef* _htim, uint32_t _Channel,
        GPIOOutput& pin_disable,
        GPIOOutput& pin_unlock,
        GPIOOutput& pin_dir_down,
        GPIOOutput& pin_pulse)
        : name_(name)
        , htim(_htim)
        , Channel(_Channel)
        , pin_disable_(pin_disable)
        , pin_unlock_(pin_unlock)
        , pin_dir_down_(pin_dir_down)
        , pin_pulse_(pin_pulse) {
    }

    // 硬件初始化
    void init() {
        // disable motor
        pin_disable_.on();
        // lock lift
        pin_unlock_.off();
        HAL_TIM_Base_Start(htim);
    }

    int set_pulse_period(int32_t pulse_period) {
        if (pulse_period <= 1) {
            return -1;
        }
        pulse_period_ = pulse_period;
        printf("%s: period=%d\n", name_, pulse_period_);
        return 0;
    }

    int start_move(int32_t pulse_count, decltype(stop_callback_) stop_callback = decltype(stop_callback_)()) {
        do {
            InterruptLock lock;
            InterruptLockGuard lock_guard(lock);
            if (moving_) {
                return -1;
            }
            if (pulse_count == 0 || pulse_period_ <= 1) {
                return -2;
            }
            moving_ = true;
            pulse_count_ = pulse_count;
            pulse_sent_ = 0;
            stop_callback_ = stop_callback;
        
            // unlock lift
            pin_unlock_.on();
            // enable motor
            pin_disable_.off();
            // set direction
            pin_dir_down_.set(pulse_count < 0);
            // pulse low
            pin_pulse_.off();
            pulse_high_ = false;
        
            __HAL_TIM_SET_COMPARE(htim, Channel, __HAL_TIM_GET_COUNTER(htim) + pulse_period_);
        
            HAL_TIM_OC_Start_IT(htim, Channel);
        } while (0);

        printf("%s: Start %d %d\n", name_, pulse_count_, pulse_period_);
        return 0;
    }
    bool is_moving() {
        return moving_;
    }
    bool get_status(int32_t& pulse_sent, int32_t& pulse_remain) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        pulse_sent   = pulse_sent_;
        pulse_remain = pulse_count_ - pulse_sent;
        return moving_;
    }
    int stop_move() {
        do {
            InterruptLock lock;
            InterruptLockGuard lock_guard(lock);
            
            if (!moving_) {
                return -1;
            }
            // pulse low
            pin_pulse_.off();
            pulse_high_ = false;
            // enable motor
            //        _enable_motor(false);
    
            __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
            HAL_TIM_OC_Stop_IT(htim, Channel);
            moving_ = false;
        } while (0);
    
        if (stop_callback_) {
            stop_callback_(pulse_sent_);
            stop_callback_ = decltype(stop_callback_)();
        }
        
        printf("%s: Stop %d/%d\n", name_, pulse_sent_, pulse_count_);
        return 0;
    }

    // 定时器回调
    void on_tim_oc_elapsed() {
        if (!moving_) {
            return;
        }
        if (pulse_count_ != 0 && pulse_sent_ != pulse_count_) {
            __HAL_TIM_SET_COMPARE(htim, Channel, __HAL_TIM_GET_COUNTER(htim) + pulse_period_ / 2);
            if (pulse_high_) {
                if (pulse_count_ > 0) {
                    pulse_sent_++;
                } else {
                    pulse_sent_--;
                }
            }
            pulse_high_ = !pulse_high_;
            pin_pulse_.set(pulse_high_);
        } else {
            stop_move();
        }
        //        printf("%s: %d/%d\n", name_, pulse_sent_, pulse_count_);
    }

    void update() {
        uint32_t now_ms = HAL_GetTick();
        if (moving_) {
            last_moving_ = now_ms;
        }
        if (last_moving_ != 0 && int32_t(now_ms - last_moving_) >= delay_lock_) {
            // disable motor
            pin_disable_.on();
            // lock lift
            pin_unlock_.off();
            last_moving_ = 0;
        }
    }
};
}