/*
 * OutputBufferUARTDMA.hpp
 *
 *  Created on: 2018年5月31日
 *      Author: shuixiang
 */

#pragma once

#include <cstdarg>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <functional>
#include <list>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"

#include "gpio.hpp"
#include "high_resolution_clock.h"
#include "utils.hpp"

namespace hustac {

class DMABuffer_UART;

//extern DMABuffer_UART UART_ROSSERIAL;
//extern DMABuffer_UART UART_TERMINAL;
//extern DMABuffer_UART UART_HEAD;
//extern DMABuffer_UART RS485_ARM_LEFT;
//extern DMABuffer_UART RS485_ARM_RIGHT;
//extern DMABuffer_UART RS485_HAND;

#define terminal UART_TERMINAL

// 基于DMA+FIFO缓冲区的UART
// 支持半双工模式, 半双工模式下将在发送过程中, 自动禁用接收, 避免回环
class DMABuffer_UART {
    /* 全局函数, 统一管理所有硬件回调 */
private:
    static std::list<DMABuffer_UART *> instances; // 用于记录所有实例
public:
    static void init_uart(const char *name_ = NULL);    // 初始化指定的UART
    static DMABuffer_UART *get_uart(const char *name_); // 初始化指定的UART
    static void on_uart_rx_abort_complete(UART_HandleTypeDef *huart_);
    static void on_uart_error(UART_HandleTypeDef *huart_);      // 当发生UART错误时, 由HAL层调用
    static void on_rx_dma_complete(UART_HandleTypeDef *huart_); // 当完成DMA接收时, 由HAL层调用
    static void on_tx_dma_complete(UART_HandleTypeDef *huart_); // 当完成DMA发送时, 由HAL层调用
    
    /* 成员变量 */
public:
    const char *name_;          // 名称
    UART_HandleTypeDef *huart_; // 对应的huart_
    /* 启用半双工通信 */
    bool half_duplix_;
    GPIOOutput *write_enable_pin_ = NULL; // 写入使能输出引脚
    
    /* Tx 发送 */
    uint8_t *tx_buf_;                        // 发送缓冲区
    const size_t tx_buf_len_;                // 发送缓冲区长度
    volatile size_t tx_write_count_ = 0; // 发送缓冲区写入索引
    volatile size_t tx_dma_start_count_ = 0; // 当前DMA发送的起始位置
    volatile size_t tx_dma_length_ = 0; // 当前DMA发送的长度
    
    size_t tx_callback_index_; // 发送回调阈值
    std::function<void()> tx_callback_;
    
    /* Rx 接收 */
    uint8_t *rx_buf_;                  // 接收缓冲区
    const size_t rx_buf_len_;          // 接收缓冲区长度
    volatile bool enable_read_ = true; // 禁用读取
    
    volatile bool rx_dma_circular_ = false;
    volatile uint32_t rx_read_count_ = 0; // 接收缓冲区读取计数
    volatile size_t rx_dma_count_ = 0; // DMA接收总数
    
    volatile size_t rx_dma_start_count_ = 0; // 当前DMA接收起始位置
    volatile size_t rx_dma_length_ = 0; // 当前DMA接收长度
    
    uint32_t last_dma_tx_ = 0;
    uint32_t count_uart_error_ = 0;
    
    size_t rx_callback_count_;          // 接收回调阈值
    std::function<void()> rx_callback_; // 接收回调
    
    std::atomic_flag mutex_rx_state_;
    volatile bool retry_rx_update_ = false;
    
    /* 成员函数 */
public:
    // 以全双工模式初始化
    DMABuffer_UART(const char *_name, UART_HandleTypeDef *_huart, uint8_t *_tx_buf, size_t _tx_buf_len,
                   uint8_t *_rx_buf, size_t _rx_buf_len)
        : name_(_name), huart_(_huart), half_duplix_(false), tx_buf_(_tx_buf), tx_buf_len_(_tx_buf_len),
          rx_buf_(_rx_buf), rx_buf_len_(_rx_buf_len) {
        instances.push_back(this);
    }
    
    // 半双工模式初始化
    DMABuffer_UART(const char *_name,
                   UART_HandleTypeDef *_huart,
                   GPIOOutput *_write_enable_pin,
                   uint8_t *_tx_buf, size_t _tx_buf_len, uint8_t *_rx_buf, size_t _rx_buf_len)
        : name_(_name), huart_(_huart), half_duplix_(true), tx_buf_(_tx_buf), tx_buf_len_(_tx_buf_len),
          rx_buf_(_rx_buf), rx_buf_len_(_rx_buf_len), write_enable_pin_(_write_enable_pin) {
        instances.push_back(this);
    }
    
    // 设定数据接收回调, 以当前接收缓冲区读出位置为基准, 当接收缓冲区的可以用数据数量>=给定阈值时, 立刻调用回调
    // 注意: 可能在中断中调用回调函数, 也可能[立即]调用回调
    int set_rx_callback(size_t count, std::function<void()> callback);
    // 可读取的字节数
    int get_rx_available();
    // 读取一个字节
    // 出错时返回-1
    int get();
    // 读取但是不取走一个字符, 无范围检查
    uint8_t _peek_unsafe(size_t index = 0) const;
    const uint8_t &operator[](size_t index) const;
    // 读取(但不取走)一个字符
    int peek(size_t index = 0);
    // 读取一块数据
    int read(uint8_t *data, size_t count);
    // 读取所有可用的数据
    int readsome(uint8_t *data, size_t max_len);
    
    // 尝试刷新输出缓冲区
    int flush();
    // 获取当前可用的输出缓冲区尺寸
    int get_tx_available();
    // 获取等待发送的数据数量
    int get_tx_sending();
    // 写入一块数据
    int write(const uint8_t *data, size_t count, bool auto_flush = true);
    // 写入字符串
    int write_string(const char *str, bool auto_flush = true);
    // 格式化输出
    // 返回实际输出的字符数, 失败则返回0
    template<int max_len = 20>
    int nprintf(const char *fmt...);

private:
    int _init() {
        tx_write_count_ = 0;
        rx_read_count_ = 0;
        rx_dma_start_count_ = 0;
        rx_dma_length_ = 0;
        
        // interrupt of both UART and DMA should be enabled
        // Tx DMA should be normal mode
        // Rx DMA should be circular mode
        // start UART Rx DMA
        // 默认状态下使能数据接收
        resume_rx();
        _start_rx_dma();
        return 0;
    }
    
    /* 接收DMA底层控制 */
    
    // 启动DMA接收, 需要保证接收DMA处于空闲状态
    int _start_rx_dma(size_t count = 0) {
        if (!enable_read_) {
            return 0;
        }
        if (huart_->RxState != HAL_UART_STATE_READY) {
            return -1;
        }
        
        _update_rx_dma_count();
        
        uint32_t start_index = rx_dma_count_ % rx_buf_len_;
        
        if (count <= 0 || count > rx_buf_len_ - start_index) {
            // count 非法时, 自动计算最大接收长度
            count = rx_buf_len_ - start_index;
            if (rx_callback_ && int32_t(rx_dma_count_ + count - rx_callback_count_) > 0) {
                count = rx_callback_count_ - rx_dma_count_;
            }
        }
        
        if (start_index == 0 && count == rx_buf_len_ && !rx_callback_) {
            // 设定接收DMA工作在环形模式
            rx_dma_circular_ = true;
            huart_->hdmarx->Init.Mode = DMA_CIRCULAR;
            uint32_t temp = huart_->hdmarx->Instance->CR;
            temp &= ~DMA_SxCR_CIRC_Msk;
            temp |= DMA_CIRCULAR;
            huart_->hdmarx->Instance->CR = temp;
        } else {
            // 设定接收DMA工作在正常模式
            rx_dma_circular_ = false;
            huart_->hdmarx->Init.Mode = DMA_NORMAL;
            uint32_t temp = huart_->hdmarx->Instance->CR;
            temp &= ~DMA_SxCR_CIRC_Msk;
            temp |= DMA_NORMAL;
            huart_->hdmarx->Instance->CR = temp;
        }
        
        HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(huart_, rx_buf_ + start_index, (uint16_t) count);
        if (ret != HAL_OK) {
            return -1;
        } else {
            //            if (strcmp(name_, "RS485_ARM_RIGHT") == 0) {
            //                printf("DMA: %d + %d\n", start_index, count);
            //            }
            rx_dma_start_count_ = rx_dma_count_;
            rx_dma_length_ = count;
            return count;
        }
    }
    
    // 停止数据接收
    int _stop_rx_dma() {
        _update_rx_dma_count();
        rx_dma_start_count_ = rx_dma_count_;
        rx_dma_length_ = 0;
        
        if (HAL_UART_AbortReceive_IT(huart_) != HAL_OK) {
            // 无法停止数据接收
            return -1;
        } else {
            return 0;
        }
    }
    
    // 获取当前DMA读取索引
    //    size_t _get_rx_dma_index() const {
    //        return (rx_dma_start_index_ + (rx_dma_length_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx))) % rx_buf_len_;
    //    }
    
    // 获取并更新当前DMA读取计数
    uint32_t _update_rx_dma_count() {
        InterruptLock lock;
        std::unique_lock<InterruptLock> u_lock(lock);
        // 获取DMA接收的索引
        uint32_t rx_dma_index;
        if (rx_dma_length_) {
            rx_dma_index =
                (rx_dma_start_count_ + (rx_dma_length_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx))) % rx_buf_len_;
        } else {
            rx_dma_index = rx_dma_start_count_ % rx_buf_len_;
        }
        // 计算上一次DMA接收的索引
        uint32_t last_rx_dma_index = rx_dma_count_ % rx_buf_len_;
        if (last_rx_dma_index == rx_dma_index) {
            // 未接收到新数据
            // todo: 正好接收了一整圈?
        } else {
            // 接收到新数据
            if (last_rx_dma_index < rx_dma_index) {
                // 未超过buf_len
                rx_dma_count_ += rx_dma_index - last_rx_dma_index;
            } else {
                // 超过buf_len
                rx_dma_count_ += rx_buf_len_ - last_rx_dma_index + rx_dma_index;
            }
        }
        
        if (rx_callback_ && int32_t(rx_dma_count_ - rx_callback_count_) >= 0) {
            // 回调接收完成
            u_lock.unlock();
            rx_callback_();
            rx_callback_ = std::function<void()>();
        }
        
        return rx_dma_count_;
    }
    
    /* 半双工通信部分 */
    
    // 使能写
    void pause_rx() {
        if (!half_duplix_)
            return;
        if (write_enable_pin_) {
            write_enable_pin_->on();
        }
        if (enable_read_) {
            enable_read_ = false;
            _stop_rx_dma();
        }
    }
    
    // 使能读
    void resume_rx() {
        if (!half_duplix_)
            return;
        if (write_enable_pin_) {
            write_enable_pin_->off();
        }
        if (!enable_read_) {
            enable_read_ = true;
            _start_rx_dma();
        }
    }
    
    void _on_uart_rx_abort_complete(UART_HandleTypeDef *_huart) {
        if (_huart == huart_) {
            _start_rx_dma();
            //            _update_rx_dma_count();
            //            rx_dma_start_count_ = rx_dma_count_;
            //            rx_dma_length_ = 0;
            //            _update_rx();
        }
    }
    
    // 当发生UART错误时, 被外部调用
    void _on_uart_error(UART_HandleTypeDef *_huart) {
        if (_huart == huart_) {
            _start_rx_dma();
            count_uart_error_++;
            //            _update_rx_dma_count();
            //            rx_dma_start_count_ = rx_dma_count_;
            //            rx_dma_length_ = 0;
            //
//            printf("%s: Error, rx_read_count_=%d rx_dma_count_=%d\n", name_, rx_read_count_, rx_dma_count_);
            //            _update_rx();
        }
    }
    
    // 当完成UART DMA接收时(非环形模式), 被外部调用
    void _on_rx_dma_complete(UART_HandleTypeDef *_huart) {
        if (_huart == huart_) {
            _update_rx_dma_count();
            if (!rx_dma_circular_) {
                _start_rx_dma();
            }
            //            printf("%s: _on_rx_dma_complete rx_dma_count_=%d\n", name_, rx_dma_count_);
        }
    }
    
    // 当完成UART DMA发送时, 被外部调用
    void _on_tx_dma_complete(UART_HandleTypeDef *_huart) {
        if (_huart == huart_) {
            //            if (strcmp(name_, "RS485_ARM_RIGHT") == 0) {
            //                printf("OK\n");
            //            }
            last_dma_tx_ = 0;
            flush();
        }
    }
};

// 设定数据接收回调, 以当前接收缓冲区读出位置为基准, 当接收缓冲区的可以用数据数量>=给定阈值时, 立刻调用回调
// 注意: 可能在中断中调用回调函数, 也可能[立即]调用回调
inline int DMABuffer_UART::set_rx_callback(size_t count, std::function<void()> callback) {
    if (!callback || rx_callback_) {
        return -1;
    }
    rx_callback_count_ = rx_read_count_ + count;
    rx_callback_ = callback;
    if (rx_dma_start_count_ + rx_dma_length_ > rx_callback_count_) {
        _stop_rx_dma();
    }
    return 0;
}

// 可读取的字节数
inline int DMABuffer_UART::get_rx_available() {
    _update_rx_dma_count();
    return rx_dma_count_ - rx_read_count_;
}

// 读取一个字节
// 出错时返回-1
inline int DMABuffer_UART::get() {
    int available = get_rx_available();
    if (available > 0) {
        uint8_t ch = rx_buf_[rx_read_count_ % rx_buf_len_];
        rx_read_count_++;
        return ch;
    } else {
        return -1;
    }
}

// 读取但是不取走一个字符, 无范围检查
inline uint8_t DMABuffer_UART::_peek_unsafe(size_t index) const {
    return rx_buf_[(rx_read_count_ + index) % rx_buf_len_];
}
inline const uint8_t &DMABuffer_UART::operator[](size_t index) const {
    return rx_buf_[(rx_read_count_ + index) % rx_buf_len_];
}

// 读取(但不取走)一个字符
inline int DMABuffer_UART::peek(size_t index) {
    if (get_rx_available() > (int) index) {
        return _peek_unsafe(index);
    } else {
        return -1;
    }
}

// 读取一块数据
inline int DMABuffer_UART::read(uint8_t *data, size_t count) {
    if (count == 0) {
        return 0;
    }
    if (get_rx_available() >= (int) count) {
        if (data != NULL) {
            uint32_t rx_read_index = rx_read_count_ % rx_buf_len_;
            if (rx_read_index + count < rx_buf_len_) {
                memcpy(data, &rx_buf_[rx_read_index], count);
            } else {
                memcpy(data, &rx_buf_[rx_read_index],
                       rx_buf_len_ - rx_read_index);
                memcpy(data + rx_buf_len_ - rx_read_index, &rx_buf_[0],
                       rx_read_index + count - rx_buf_len_);
            }
        }
        rx_read_count_ += count;
        return count;
    } else {
        return -1;
    }
}

// 读取所有可用的数据
inline int DMABuffer_UART::readsome(uint8_t *data, size_t max_len) {
    int count = get_rx_available();
    if (count <= 0) {
        return 0;
    }
    if (data != NULL) {
        uint32_t rx_read_index = rx_read_count_ % rx_buf_len_;
        if (rx_read_index + count < rx_buf_len_) {
            memcpy(data, &rx_buf_[rx_read_index], count);
        } else {
            memcpy(data, &rx_buf_[rx_read_index],
                   rx_buf_len_ - rx_read_index);
            memcpy(data + rx_buf_len_ - rx_read_index, &rx_buf_[0],
                   rx_read_index + count - rx_buf_len_);
        }
    }
    rx_read_count_ += count;
    return count;
}

/* 写入部分, 允许在中断中使用 */

//    volatile uint32_t debug_i = 0;

// 尝试刷新输出缓冲区
inline int DMABuffer_UART::flush() {
    int ret;
    //    uint64_t now = MY_GetNanoSecFromCycle(MY_GetCycleCount());
    
    InterruptLock lock;
    std::unique_lock<InterruptLock> u_lock(lock);
    if (huart_->gState != HAL_UART_STATE_READY) {
        // DMA 忙
        if (last_dma_tx_ != 0 && HAL_GetTick() - last_dma_tx_ > 1000) {
            //                printf("TO\n");
            last_dma_tx_ = 0;
            u_lock.unlock();
            printf("%s: to %d\n", name_, HAL_GetTick() - last_dma_tx_);
        }
        //            if (debug_i != 0 && strcmp(name_, "RS485_ARM_RIGHT") == 0) {
        ////                printf("no ready\n");
        //                debug_i = 0;
        //            }
        ret = -1;
    } else {
        // 可以发送数据
        tx_dma_start_count_ += tx_dma_length_;
        tx_dma_length_ = 0;
        
        if (int32_t(tx_write_count_ - tx_dma_start_count_) <= 0) {
            // 没有待发送的数据
            //            if (debug_i != 2 && strcmp(name_, "RS485_ARM_RIGHT") == 0) {
            //                printf("no need\n");
            //                debug_i = 2;
            //            }
            resume_rx();
            ret = 0;
        } else {
            // 启动DMA发送
            size_t block_len;
            int tx_write_index = tx_write_count_ % tx_buf_len_;
            int tx_dma_index = tx_dma_start_count_ % tx_buf_len_;
            if (tx_write_index <= tx_dma_index) {
                // 需要分为两次发送
                block_len = tx_buf_len_ - tx_dma_index;
            } else {
                // 一次即可发送
                block_len = tx_write_count_ - tx_dma_start_count_;
            }
            pause_rx();
            //            while (true) {
            int ret = HAL_UART_Transmit_DMA(huart_, &(tx_buf_[tx_dma_index]), block_len);
            if (ret == HAL_OK) {
                tx_dma_length_ = block_len;
                ret = block_len;
                last_dma_tx_ = HAL_GetTick();
                //                    if (strcmp(name_, "RS485_ARM_RIGHT") == 0) {
                //                        printf("S.", block_begin_index, block_begin_index + block_len);
                //                    }
                //                    break;
            } else {
                u_lock.unlock();
                printf("%s: lck\n", name_);
                ret = -2;
            }
        }
    }
    return ret;
}

// 获取当前可用的输出缓冲区尺寸
inline int DMABuffer_UART::get_tx_available() {
    return tx_buf_len_ - get_tx_sending();
}

// 获取等待发送的数据数量
inline int DMABuffer_UART::get_tx_sending() {
    InterruptLock lock;
    InterruptLockGuard lock_guard(lock);
    size_t tx_dma_remain = __HAL_DMA_GET_COUNTER(huart_->hdmatx);
    return tx_write_count_ - (tx_dma_start_count_ + tx_dma_length_ - tx_dma_remain);
}

// 写入一块数据
inline int DMABuffer_UART::write(const uint8_t *data, size_t count, bool auto_flush) {
    int ret;
    do {
        InterruptLock lock;
        InterruptLockGuard lock_guard(lock);
        if (get_tx_available() > count) {
            size_t begin_index = tx_write_count_ % tx_buf_len_;
        
            if (begin_index + count < tx_buf_len_) {
                std::memcpy(&tx_buf_[begin_index], data, count);
            } else {
                std::memcpy(&tx_buf_[begin_index], data,
                            tx_buf_len_ - begin_index);
                std::memcpy(&tx_buf_[0], data + tx_buf_len_ - begin_index,
                            begin_index + count - tx_buf_len_);
            }
        
            tx_write_count_ += count;
        
            ret = count;
            //            if (strcmp(name_, "RS485_ARM_RIGHT") == 0) {
            //                printf("write: OK %d-%d\n", begin_index, begin_index + count);
            //            }
        } else {
            ret = -1;
        }
    } while (0);
    
    if (auto_flush) {
        flush();
    }
    return ret;
}

// 写入字符串
inline int DMABuffer_UART::write_string(const char *str, bool auto_flush) {
    if (str) {
        return write((const uint8_t *) str, strlen(str), auto_flush);
    } else {
        return -1;
    }
}

// 格式化输出
// 返回实际输出的字符数, 失败则返回0
template<int max_len>
inline int DMABuffer_UART::nprintf(const char *fmt...) {
    uint8_t buf[max_len];
    std::va_list args;
    va_start(args, fmt); //获得可变参数列表
    int n = vsnprintf((char *) buf, max_len, fmt, args);
    va_end(args); //释放资源
    if (n + 1 > max_len) {
        // 未写入全部数据
        int ret = write(buf, max_len);
        return ret;
    } else if (n > 0) {
        // 全部写入
        int ret = write(buf, n);
        return ret;
    } else if (n == 0) {
        // 未写入任何数据
        return 0;
    } else {
        return n;
    }
}
}
