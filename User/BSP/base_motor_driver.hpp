#pragma once

#include <array>

#include "base_motor.hpp"
#include "dmabuffer_uart.hpp"

namespace hustac {

class AbstractMotorDriver {
public:
    static const uint32_t default_delay_init = 2000;    // 上电后延迟启动(ms)
    static const uint64_t default_interval_send = 100000;  // 帧发送最小间隔(ns)
    static const uint64_t default_rx_resp_timeout_ns = 500000;  // 帧等待超时(ns)
    static const uint64_t default_rx_interval_timeout_ns = 500000;  // 帧接收字符超时(ns)
    static const uint64_t default_dur_loop = 1000000; // 状态机周期(ns)
    static const uint32_t default_disconnect_timeout = 100; // 超过该事件后未响应, 则认为连接断开(ms)
    
    bool ignore_timeout_ = false;
    bool ignore_busy_ = false;
    
    uint32_t delay_init_ = default_delay_init;
    uint32_t disconnect_timeout_ = default_disconnect_timeout;
    uint64_t interval_send_ = default_interval_send;
    uint64_t rx_resp_timeout_ns_ = default_rx_resp_timeout_ns;
    uint64_t rx_interval_timeout_ns_ = default_rx_interval_timeout_ns;
    uint64_t dur_loop_ = default_dur_loop;
    
    // 总状态
    enum class State {
        WAIT,               // 定时1ms
        SYNC_WRITE,         // 写入设定位置
        SYNC_WRITE_SENDING, // 等待数据发送完毕
        REQUEST,            // 请求读取当前位置
        REQUEST_SENDING,    // 等待请求发送完毕
        WAIT_RESPONSE,      // 等待当前位置响应
        PENDING_RESPONSE,   // 接收响应
    };
    
    const char* name_;
    DMABuffer_UART& uart_;
    
    bool connected_ = false;
    volatile State state_ = State::WAIT; // 当前总状态
    
    volatile bool current_external_op_ = false; // 当前正在执行外部操作
    volatile bool current_op_success_ = true;
    volatile uint8_t current_operation_index_ = 1; // 当前操作的序号
    volatile uint32_t rx_frame_param_count_ = 0; // 接收到的参数个数
    
    SoftTimerNS<0, false> timer_loop_ = {0}; // 状态机循环定时器
    SoftTimerMS<100> timer_print_timeout_ = {0}; // 错误打印定时器
    uint64_t last_sync_op_sending_ = 0;     // 上一次sync_op开始发送时间(ns)
    uint64_t last_sync_op_sent_ = 0;     // 上一次sync_op发送完成时间(ns)
    uint64_t last_op_sending_ = 0;     // 上一次op开始发送时间(ns)
    uint64_t last_op_sent_ = 0;     // 上一次op发送完成的时间(ns)
    uint64_t last_recv_start_ = 0;     // 上一次接收到帧首字节的时间(ns)
    uint64_t last_data_recved_ = 0;     // 上一次接收到串口数据的时间(ns)
    uint64_t last_recv_ok_ = 0;     // 上一次接收到完整帧的时间(ns)
    
    uint32_t dur_send_sync_op_; // 发送sync_op耗时(ns)
    uint32_t dur_send_op_;      // 发送op耗时(ns)
    uint32_t dur_wait_resp_;    // 等待响应耗时(ns)
    uint32_t dur_recv_resp_;    // 接收响应耗时(ns)
    
    uint32_t count_sync_op_ = 0; // 统计同步写入(无响应)的次数
    uint32_t count_op_ = 0; // 统计成功(有响应地)读写数据的次数
    uint32_t count_read_ = 0; // 统计成功读取位置/速度/力矩的次数
    uint32_t count_error_ = 0; // 统计发生错误的次数
    
    // 需要进行同步写指令之前的回调
    std::function<void(AbstractMotorDriver&)> sync_write_callback_;
    // 读取完成所有电机的数据后的回调
    std::function<void(AbstractMotorDriver&)> read_pos_callback_;
    
    AbstractMotorDriver(const char* name, DMABuffer_UART& rs485)
        : name_(name), uart_(rs485) {
        timer_loop_.set_interval(dur_loop_);
    }
    
    void set_sync_write_callback(decltype(sync_write_callback_) sync_write_callback) {
        auto pri = taskENTER_CRITICAL_FROM_ISR();
        sync_write_callback_ = sync_write_callback;
        taskEXIT_CRITICAL_FROM_ISR(pri);
    }
    
    void set_read_pos_callback(decltype(sync_write_callback_) read_pos_callback) {
        auto pri = taskENTER_CRITICAL_FROM_ISR();
        read_pos_callback_ = read_pos_callback;
        taskEXIT_CRITICAL_FROM_ISR(pri);
    }
    
    bool connected() {
        return connected_;
    }
    
    virtual int motor_size() = 0;
    virtual BaseMotor& motor(int id) = 0;
    // 协议解析, 返回0表示成功, <0表示出错
    virtual int _parse_update(uint8_t ch) = 0;
    virtual void _generate_operation() = 0;
    virtual int _do_operation() = 0;
    virtual void _generate_sync_operation() = 0;
    virtual int _do_sync_operation() = 0;
    
protected:
    void on_init_finished() {
        connected_ = true;
        printf("%s: All Connected\n", name_);
    }
};

template<typename OperationT, typename SyncOperationT>
class BaseMotorDriver : public AbstractMotorDriver {

public:
    OperationT current_op_;
    SyncOperationT current_sync_op_;
    
    // 待执行的操作队列
    static const uint32_t max_pending_operations_ = 20;
    QueueSafe<OperationT, max_pending_operations_> pending_operations_;

public:
    BaseMotorDriver(const char* name, DMABuffer_UART& rs485)
        : AbstractMotorDriver(name, rs485) {
    }
    
    // 添加一个等待执行的操作
    // 返回: 成功返回待执行的操作数, 失败返回-1
    int add_operation(const OperationT& op) {
        int ret = pending_operations_.push(op);
        if (ret < 0) {
            printf("%s: failed to add_operation");
        }
        return ret;
    }
    
    // 是否有操作待执行
    bool is_operation_pending() {
        return !pending_operations_.empty();
    }
    
    // 通信状态机
    void spin_once() {
        uint64_t current_ns = MY_GetNanoSecFromCycle(MY_GetCycleCount());
        int periods;
        switch (state_) {
        case State::WAIT:
            // 等待上电延迟
            if (HAL_GetTick() < delay_init_) {
                break;
            }
            // 周期等待
            periods = timer_loop_.is_timeout();
            if (periods) {
                if (!ignore_busy_ && count_read_ > 1 && periods > 1 && timer_loop_.count_ > periods) {
                    printf(
                        "%s: Warning! Too busy, %d periods escaped, dur_send_sync_op_=%d, dur_send_op_=%d, dur_wait_resp_=%d, dur_recv_resp_=%d\n",
                        name_, periods, dur_send_sync_op_ / 1000, dur_send_op_ / 1000, dur_wait_resp_ / 1000,
                        dur_recv_resp_ / 1000);
                }
                state_ = State::SYNC_WRITE;
            } else {
                state_ = State::WAIT;
            }
            break;
        case State::SYNC_WRITE:
            // 准备发送数据 同步写各关节目标位置
            if (sync_write_callback_) {
                sync_write_callback_(*this);
            }
            _generate_sync_operation();
            _do_sync_operation();
            last_sync_op_sending_ = current_ns;
            
            state_ = State::SYNC_WRITE_SENDING;
            break;
        case State::SYNC_WRITE_SENDING:
            // 正在发送请求 同步写各关节目标位置
            if (uart_.get_tx_sending() == 0) {
                count_sync_op_++;
                dur_send_sync_op_ = current_ns - last_sync_op_sending_;
                last_sync_op_sent_ = current_ns;
                state_ = State::REQUEST;
            }
            break;
        case State::REQUEST:
            // 准备发送读请求 读取当前位置
            if (current_ns - last_op_sent_ >= interval_send_) {
                if (current_op_success_) {
                    _generate_operation();
                }
                _do_operation();
                current_op_success_ = false;
                last_op_sending_ = current_ns;
                
                state_ = State::REQUEST_SENDING;
            }
            break;
        case State::REQUEST_SENDING:
            // 正在发送读请求 读取当前位置
            if (uart_.get_tx_sending() == 0) {
                dur_send_op_ = current_ns - last_op_sending_;
                last_op_sent_ = current_ns;
                state_ = State::WAIT_RESPONSE;
            }
            break;
        case State::WAIT_RESPONSE:
            // 等待响应的第一个字节 (读取当前位置)
            if (uart_.get_rx_available() > 0) {
                dur_wait_resp_ = current_ns - last_op_sent_;
                last_data_recved_ = current_ns;
                last_recv_start_ = current_ns;
                state_ = State::PENDING_RESPONSE;
            } else {
                if (current_ns - last_op_sent_ >= rx_resp_timeout_ns_) {
                    // 限制错误信息的最高打印频率
                    if (!ignore_timeout_ && timer_print_timeout_.is_timeout()) {
                        printf("%s[%d]: op wait resp timeout (%d us)\n", name_, current_op_.id,
                               (int) ((current_ns - last_op_sent_) / 1000));
                    }
                    count_error_++;
                    state_ = State::WAIT; // 读取失败后切换状态
                }
            }
            break;
        case State::PENDING_RESPONSE:
            // 接收完整的响应帧 (读取当前位置)
            while (true) {
                int ch = uart_.get();
                if (ch >= 0) {
                    last_data_recved_ = current_ns;
                    int ret = _parse_update(ch);
                    if (ret == 0) {
                        // 若得到完整的数据包, 存储关节位置信息
                        dur_recv_resp_ = current_ns - last_recv_start_;
                        // 等待响应平均耗时0.2ms
                        //                        printf("%s[%d]: op ok, dur_wait_resp_=%d us\n", name_, current_op_.id, dur_wait_resp_ / 1000);
                        
                        count_op_++;
                        current_op_success_ = true;
                        last_recv_ok_ = current_ns;
                        
                        if (current_op_.on_finish) {
                            current_op_.on_finish(current_op_);
                            current_op_.on_finish = decltype(current_op_.on_finish)();
                        }
                        state_ = State::WAIT; // 读取成功后切换状态
                        break;
                    } else if (ret < 0) {
                        // 接收出错
                        count_error_++;
                        state_ = State::WAIT; // 读取成功后切换状态
                        break;
                    } else {
                        // 还在接收
                    }
                } else {
                    if (current_ns - last_data_recved_ >= rx_interval_timeout_ns_) {
                        // 读取超时
                        if (!ignore_timeout_ && timer_print_timeout_.is_timeout()) {
                            printf("%s[%d]: op interval timeout (%d us)\n", name_, current_operation_index_,
                                   int((current_ns - last_data_recved_) / 1000));
                        }
                        count_error_++;
                        state_ = State::WAIT; // 读取失败后切换状态
                    }
                    break;
                }
                current_ns = MY_GetNanoSecFromCycle(MY_GetCycleCount());
            }
            break;
        default:
            break;
        }
        
        if (connected_ && (last_recv_ok_ == 0 ||
                           (MY_GetNanoSecFromCycle(MY_GetCycleCount()) - last_recv_ok_) / 1000 > disconnect_timeout_))
            connected_ = false;
    }
};
}
