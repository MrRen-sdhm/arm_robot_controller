#pragma once

#include <cstring>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <functional>
#include <stdexcept>
#include <type_traits>

#include "stm32f4xx_hal.h"

#include "high_resolution_clock.h"

#include "mutex.hpp"
#include "pipe.hpp"
#include "pipe_tcp_client.hpp"
#include "pipe_tcp_server.hpp"
#include "pipe_udp_broadcast.hpp"

namespace hustac {

struct null_type {
    struct value_type {
    };
    value_type value;
};

// 软件固定间隔定时(定数)器
template<typename T, std::conditional_t<std::is_integral<T>::value, T, uint64_t> interval = 0, bool skip = true>
class SoftTimer {
public:
    uint32_t count_ = 0; // 成功执行次数
    T last_count_;
    // 当interval=0时启用
    std::conditional_t<interval <= 0, T, null_type> interval_;
    std::function<T()> func_get_count_;
    
    SoftTimer()
        : SoftTimer(default_counter()()) {}
    
    SoftTimer(T init_count)
        : SoftTimer(init_count, default_counter()) {}
    
    SoftTimer(std::function<T()> func_get_count)
        : SoftTimer(func_get_count(), func_get_count) {}
    
    SoftTimer(T init_count, std::function<T()> func_get_count)
        : last_count_(init_count), func_get_count_(func_get_count) {}
    
    // 重新从现在开始计时
    T reset() {
        last_count_ = get_count();
        return last_count_;
    }
    
    // 从上一次更新到现在的时长
    T duration() {
        return get_count() - last_count_;
    }
    
    // 当interval=0时启用
    template<decltype(interval) U = interval, std::enable_if_t<U == 0, int> = 0>
    void set_interval(T d) {
        interval_ = d;
    }
    
    // 检查是否超时
    // 保证平均执行时间差接近给定时间间隔
    int is_timeout(std::function<bool(uint32_t count)> callback = std::function<bool(uint32_t count)>()) {
        T now = get_count();
        T duration = now - last_count_;
        if (get_interval() == 0) {
            if (!callback || callback(count_)) {
                count_++;
            }
            last_count_ = now;
            return 1;
        } else {
            uint32_t periods = uint32_t(duration / get_interval());
            if (skip) {
                // 避免连续重复执行, 实际执行频率可能低于期望频率
                if (periods) {
                    if (!callback || callback(count_)) {
                        count_++;
                    }
                }
            } else {
                // 实际执行频率等于期望频率
                for (uint32_t i = 0; i < periods; i++) {
                    if (!callback || callback(count_)) {
                        count_++;
                    }
                }
            }
            // 下一次执行在给定时间间隔之后
            last_count_ += periods * get_interval();
            return periods;
        }
    }
    
    // 获取当前计数值
    T get_count() { return func_get_count_(); }
    
    // 获取间隔值
    // 当T=float/double且interval=0时启用
    template<typename U = T, std::enable_if_t<std::is_floating_point<U>::value && interval == 0, int> = 0>
    T get_interval() { return interval_; }
    // 当T=float/double且interval!=0时启用
    template<typename U = T, std::enable_if_t<std::is_floating_point<U>::value && interval != 0, int> = 0>
    T get_interval() { return interval / 1e9; }
    // 当T!=float/double且interval=0时启用
    template<typename U = T, std::enable_if_t<std::is_integral<U>::value && interval == 0, int> = 0>
    T get_interval() { return interval_; }
    // 当T!=float/double且interval!=0时启用
    template<typename U = T, std::enable_if_t<std::is_integral<U>::value && interval != 0, int> = 0>
    T get_interval() { return interval; }

private:
    // 获取默认计时器
    // 使用enable_if_t对成员函数, 进行部分特化(条件编译)
    // 当T=uint32_t时启用, 返回当前毫秒数
    template<typename U = T, std::enable_if_t<std::is_same<U, uint32_t>::value, int> = 0>
    static std::function<T()> default_counter() {
        return HAL_GetTick;
    }
    // 当T=uint64_t时启用, 返回当前纳秒数
    template<typename U = T, std::enable_if_t<std::is_same<U, uint64_t>::value, int> = 0>
    static std::function<T()> default_counter() {
        return []() -> uint64_t {
            return MY_GetNanoSecFromCycle(MY_GetCycleCount());
        };
    }
    // 当T=loat/double时启用, 返回当前秒数
    template<typename U = T, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
    static std::function<T()> default_counter() {
        return []() -> T {
            return MY_GetNanoSecFromCycle(MY_GetCycleCount()) / 1e9f;
        };
    }
};

template<uint32_t interval = 0, bool fixed_interval = true>
using SoftTimerMS = SoftTimer<uint32_t, interval, fixed_interval>;

template<uint64_t interval = 0, bool fixed_interval = true>
using SoftTimerNS = SoftTimer<uint64_t, interval, fixed_interval>;

template<uint64_t interval = 0, bool fixed_interval = true>
using SoftTimerFloat = SoftTimer<float, interval, fixed_interval>;

template<uint64_t interval = 0, bool fixed_interval = true>
using SoftTimerDouble = SoftTimer<double, interval, fixed_interval>;

// 环形FIFO, 提供互斥保护
template<typename T, size_t max_size_>
class Queue {
public:
    Queue() {
    }
    ~Queue() {
        clear();
    }
    // 重置
    void clear() {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        while (!empty_) {
            pop();
        }
        in_ = 0;
        out_ = 0;
    }
    // 是否为空
    bool empty() const {
        return empty_;
    }
    // 当前长度
    size_t size() const {
        if (empty_) {
            return 0;
        } else {
            InterruptLock lock;
            InterruptLockGuard lock_guard(lock);
            size_t s = (in_ + max_size_ - out_) % max_size_;
            if (s == 0) {
                return max_size_;
            } else {
                return s;
            }
        }
    }
    // 最大长度
    size_t max_size() const {
        return max_size_;
    }
    // 入栈, 成功返回剩余数量, 失败返回-1
    int push(const T &item) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        if (!empty_ && in_ == out_) {
            // 队列已满
            return -1;
        }
        // 置位 new
        new(&data_[in_]) T(item);
        in_ = (in_ + 1) % max_size_;
        empty_ = false;
        return size();
    }
    // 出栈, 成功返回剩余数量, 失败返回-1
    int pop() {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        if (empty_) {
            return -1;
        }
        // 显式析构
        (&data_[out_])->~T();
        out_ = (out_ + 1) % max_size_;
        if (out_ == in_) {
            empty_ = true;
        }
        return size();
    }
    int pop(T &item) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        if (empty_) {
            return -1;
        }
        // 移动元素
        item = std::move(data_[out_]);
        // 显式析构
        (&data_[out_])->~T();
        out_ = (out_ + 1) % max_size_;
        if (out_ == in_) {
            empty_ = true;
        }
        return size();
    }
    // 插入, 支持负索引, 成功返回剩余数量, 失败返回-1
    int insert(int index, const T &item);
    // 移除, 支持负索引, 成功返回剩余数量, 失败返回-1
    int erase(int index);
    int erase(int index, T &item);
    // 随机访问, 支持负索引
    T &at(int index) {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        size_t length = size();
        if (index >= length || index < length) {
            throw std::out_of_range("");
        }
        return (*this)[index];
    }
    T &operator[](int index) {
        index = (out_ + index + max_size_) % max_size_;
        return data_[index];
    }
    const T &at(int index) const; // 有范围检查
    const T &operator[](int index) const; // 无范围检查
private:
    uint8_t memory_[sizeof(T) * max_size_];
    T *data_ = (T *) memory_;
    volatile bool empty_ = true;
    volatile size_t in_ = 0;
    volatile size_t out_ = 0;
};

template<typename T, size_t max_size>
using QueueUnsafe = Queue<T, max_size>;

template<typename T, size_t max_size>
using QueueSafe = Queue<T, max_size>;


template<typename T, typename U>
constexpr size_t offsetOf(U T::*member) {
    return (char *) &((T *) nullptr->*member) - (char *) nullptr;
}


class TicToc {
private:
    uint64_t tic_;
    uint64_t toc_ = 0;
public:
    TicToc() : tic_(MY_GetNanoSecFromCycle(MY_GetCycleCount())) {}
    TicToc(uint64_t tic_ns) : tic_(tic_ns) {}
    TicToc(float tic_sec) : tic_(uint64_t(tic_sec * 1e9f)) {}
    uint64_t start() {
        return tic_ = MY_GetNanoSecFromCycle(MY_GetCycleCount());
    }
    uint64_t stop() {
        return toc_ = MY_GetNanoSecFromCycle(MY_GetCycleCount());
    }
    uint64_t duration() {
        if (toc_ < tic_) {
            return uint64_t(-1);
        } else {
            return toc_ - tic_;
        }
    }
};

}