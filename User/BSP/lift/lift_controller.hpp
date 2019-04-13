#pragma once

#include <algorithm>
#include <array>

#include "stm32f4xx_hal.h"

#include "dmabuffer_uart.hpp"
#include "gpio.hpp"
#include "utils.hpp"
#include "step_motor.hpp"

namespace hustac {

class LiftSensor {
public:
    const char* name_;
    GPIO_TypeDef* gpio_port_;
    uint32_t gpio_pin_;
    LiftSensor(const char* name, GPIO_TypeDef* gpio_port, uint32_t gpio_pin)
        : name_(name)
        , gpio_port_(gpio_port)
        , gpio_pin_(gpio_pin) {
    }
    bool is_inside() const {
        return HAL_GPIO_ReadPin(gpio_port_, gpio_pin_) == GPIO_PIN_RESET;
    }
};

class LiftController {
public:
    const char* name_;
    const std::array<LiftSensor, 3>& sensors_;
    StepMotor& motor_;

    int32_t position_ = 0;

    LiftController(const char* name, const std::array<LiftSensor, 3>& sensors, StepMotor& motor)
        : name_(name)
        , sensors_(sensors)
        , motor_(motor) {
    }

    void init() {
        motor_.init();
    }

    int manual_move(float distance, float velocity = 0.005) {
        if (motor_.is_moving()) {
            return -1;
        }
        int32_t pulse_period = 1000000 / motor_.meter_to_pulse(velocity);
        int ret              = motor_.set_pulse_period(pulse_period);
        if (ret < 0) {
            return ret;
        }
        int32_t pulse_count = motor_.meter_to_pulse(distance);
        ret                 = motor_.start_move(pulse_count, [this](int32_t pulse_sent) {
            printf("%s: %d%+d = %d\n", name_, position_, pulse_sent, position_ + pulse_sent);
            position_ += pulse_sent;
        });
        if (ret < 0) {
            return ret;
        }
        return 0;
    }

    void update() {
        motor_.update();
        int32_t pulse_sent, pulse_remain;
        if (motor_.get_status(pulse_sent, pulse_remain)) {
            bool need_stop = false;
            if (sensors_[0].is_inside() && pulse_remain < 0) {
                printf("%s: Reach %s\n", name_, sensors_[0].name_);
                need_stop = true;
            }
            //            if (sensors_[1].is_inside()) {
            //                printf("%s: Reach %s\n", name_, sensors_[1].name_);
            //            }
            if (sensors_[2].is_inside() && pulse_remain > 0) {
                printf("%s: Reach %s\n", name_, sensors_[2].name_);
                need_stop = true;
            }
            if (need_stop) {
                motor_.stop_move();
            }
        }
    }
};

extern std::array<LiftSensor, 3> lift_sensors;
extern StepMotor lift_motor;
extern LiftController lift;
}