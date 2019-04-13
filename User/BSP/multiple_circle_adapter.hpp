#pragma once

#include <cmath>
#include <utility>

#include "dmabuffer_uart.hpp"

namespace hustac {

// 支持电机多圈位置
template <typename T = float>
class MultipleCircleAdapter {
public:
    MultipleCircleAdapter(const std::pair<T, T>& range, T init_motor_pos = 0.0f, int init_circles = 0)
        : motor_range_(range)
        , last_motor_pos_(init_motor_pos)
        , circles_(init_circles) {}

    T update(T new_motor_pos);

    T position();

    void position(T new_pos);

    int circles() { return circles_; }

    const std::pair<T, T>& motor_range() { return motor_range_; }

private:
    std::pair<T, T> motor_range_; // 电机硬件范围
    T last_motor_pos_;
    int circles_;
};

template <typename T>
inline T MultipleCircleAdapter<T>::position() {
    return circles_ * (motor_range_.second - motor_range_.first) + last_motor_pos_;
}

template <typename T>
inline void MultipleCircleAdapter<T>::position(T new_pos) {
    new_pos -= motor_range_.first;
    circles_        = int(std::floor(new_pos / (motor_range_.second - motor_range_.first)));
    last_motor_pos_ = new_pos - circles_ * (motor_range_.second - motor_range_.first);
}

template <typename T>
inline T MultipleCircleAdapter<T>::update(T new_motor_pos) {
    if (new_motor_pos - last_motor_pos_ >= (motor_range_.second - motor_range_.first) / 2) {
        // 转动超过下限
//        printf("MultiCirle: %.3f -> %.3f circles_--\n", last_motor_pos_, new_motor_pos);
        circles_--;
    } else if (new_motor_pos - last_motor_pos_ < -(motor_range_.second - motor_range_.first) / 2) {
        // 转动超过上限
//        printf("MultiCirle: %.3f -> %.3f circles_++\n", last_motor_pos_, new_motor_pos);
        circles_++;
    } else {
        // 转动未超过限制
//        printf("MultiCirlce: %.3f -> %.3f\n", last_motor_pos_, new_motor_pos);
    }
    last_motor_pos_ = new_motor_pos;
    return position();
}
}
