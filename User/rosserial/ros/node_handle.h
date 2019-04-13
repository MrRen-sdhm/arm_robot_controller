/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_NODE_HANDLE_H_
#define ROS_NODE_HANDLE_H_

#include <algorithm>
#include <functional>
#include <inttypes.h>
#include <limits>
#include <memory>
#include <stdint.h>

#include "rosserial_msgs/Log.h"
#include "rosserial_msgs/RequestParam.h"
#include "rosserial_msgs/TopicInfo.h"
#include "std_msgs/Time.h"

#include "ros/msg.h"

namespace ros {

class NodeHandleBase_ {
public:
    virtual int publish(int id, const Msg* msg) = 0;
    virtual int spinOnce() = 0;
    virtual bool connected() = 0;
    virtual ~NodeHandleBase_() {
    }
};
} // namespace ros

#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"

#include "dmabuffer_uart.hpp"
#include "utils.hpp"

namespace ros {

const int SPIN_OK = 0;
const int SPIN_ERR = -1;
const int SPIN_TIMEOUT = -2;

const uint32_t SYNC_INTERVAL = 100; // 每两次同步之间的时间间隔, ms
const uint32_t LOST_TIMEOUT = 500; // 每两次同步之间的时间间隔, ms
const uint8_t MODE_FIRST_FF = 0;
/*
 * The second sync byte is a protocol version. It's value is 0xff for the first
 * version of the rosserial protocol (used up to hydro), 0xfe for the second version
 * (introduced in hydro), 0xfd for the next, and so on. Its purpose is to enable
 * detection of mismatched protocol versions (e.g. hydro rosserial_python with groovy
 * rosserial_arduino. It must be changed in both this file and in
 * rosserial_python/src/rosserial_python/SerialClient.py
 */
const uint8_t MODE_PROTOCOL_VER = 1;
const uint8_t PROTOCOL_VER1 = 0xff; // through groovy
const uint8_t PROTOCOL_VER2 = 0xfe; // in hydro
const uint8_t PROTOCOL_VER = PROTOCOL_VER2;
const uint8_t MODE_SIZE_L = 2;
const uint8_t MODE_SIZE_H = 3;
const uint8_t MODE_SIZE_CHECKSUM = 4; // checksum for msg size received from size L and H
const uint8_t MODE_TOPIC_L = 5; // waiting for topic id
const uint8_t MODE_TOPIC_H = 6;
const uint8_t MODE_MESSAGE = 7;
const uint8_t MODE_MSG_CHECKSUM = 8; // checksum for msg and topic id

const uint32_t SERIAL_MSG_TIMEOUT = 20000; // 20 us to recieve all of message data

using rosserial_msgs::TopicInfo;

/* Node Handle */
template<class Hardware, int MAX_SUBSCRIBERS = 50, int MAX_PUBLISHERS = 50>
class NodeHandle_ : public NodeHandleBase_ {
protected:
    const char* name_;
    Hardware hardware_;
    
    /* time used for syncing */
    uint64_t rt_time;   // 请求同步时间的时间点, ns
    
    /* used for computing current time */
    uint64_t nsec_offset;
    float time_variance = -1;
    
    /* Spinonce maximum work timeout */
    uint32_t spin_timeout_;
    
    Publisher* publishers[MAX_PUBLISHERS];
    Subscriber_* subscribers[MAX_SUBSCRIBERS];
    
    uint8_t* message_in_ = NULL;
    size_t message_in_size_ = 0;
    uint8_t* message_out_ = NULL;
    size_t message_out_size_ = 0;
    
    std::function<void()> connected_callback_;
    std::function<void()> disconnected_callback_;
    
    hustac::SoftTimerMS<1000> timer_print_sync_ = {0};
    /*
     * Setup Functions
     */
public:
    NodeHandle_(const char* name)
        : name_(name), configured_(false) {
        
        for (unsigned int i = 0; i < MAX_PUBLISHERS; i++)
            publishers[i] = 0;
        
        for (unsigned int i = 0; i < MAX_SUBSCRIBERS; i++)
            subscribers[i] = 0;
        
        req_param_resp.ints_length = 0;
        req_param_resp.ints = NULL;
        req_param_resp.floats_length = 0;
        req_param_resp.floats = NULL;
        req_param_resp.ints_length = 0;
        req_param_resp.ints = NULL;
        
        spin_timeout_ = 0;
    }
    
    Hardware* getHardware() {
        return &hardware_;
    }
    
    /* Start serial, initialize buffers */
    void initNode() {
        hardware_.init();
        mode_ = 0;
        bytes_ = 0;
        index_ = 0;
        topic_ = 0;
    };
    
    /* Start a named port, which may be network server IP, initialize buffers */
    void initNode(const char* portName, uint8_t* buf_in = NULL, size_t buf_in_size = 0, uint8_t* buf_out = NULL, size_t buf_out_size = 0) {
        hardware_.init(portName);
        mode_ = 0;
        bytes_ = 0;
        index_ = 0;
        topic_ = 0;
        message_in_ = buf_in;
        message_in_size_ = buf_in_size;
        message_out_ = buf_out;
        message_out_size_ = buf_out_size;
    };
    
    virtual ~NodeHandle_() {
    }
    
    /**
     * @brief Sets the maximum time in millisconds that spinOnce() can work.
     * This will not effect the processing of the buffer, as spinOnce processes
     * one byte at a time. It simply sets the maximum time that one call can
     * process for. You can choose to clear the buffer if that is beneficial if
     * SPIN_TIMEOUT is returned from spinOnce().
     * @param timeout The timeout in milliseconds that spinOnce will function.
     */
    void setSpinTimeout(const uint32_t& timeout) {
        spin_timeout_ = timeout;
    }

protected:
    //State machine variables for spinOnce
    int mode_;
    int bytes_;
    int topic_;
    int index_;
    int checksum_;
    
    bool configured_;
    uint32_t configured_t_;
    
    /* used for syncing the time */
    uint32_t last_sync_time;
    uint32_t last_sync_receive_time;
    uint32_t last_msg_timeout_time; // 单帧消息接收超时, ms

public:
    /* This function goes in your loop() function, it handles
     *  serial input and callbacks for subscribers.
     */
    
    virtual int spinOnce() {
        /* restart if timed out */
        uint32_t c_time = hardware_.time();
        if (configured_ && c_time - last_sync_receive_time > LOST_TIMEOUT) {
            // 过长时间(2倍于同步间隔)没有收到同步时间响应
            // 认为断开连接
            configured_ = false;
            printf("NodeHandle %s: Lost (sync timeout)\n", name_);
            hardware_.init();
        }
        
        /* while available buffer, read data */
        while (true) {
            // If a timeout has been specified, check how long spinOnce has been running.
            if (spin_timeout_ > 0) {
                // If the maximum processing timeout has been exceeded, exit with error.
                // The next spinOnce can continue where it left off, or optionally
                // based on the application in use, the hardware buffer could be flushed
                // and start fresh.
                if ((hardware_.time() - c_time) > spin_timeout_) {
                    // 一次更新所耗费时间大于上限
                    // Exit the spin, processing timeout exceeded.
                    printf("NodeHandle %s: spin timeout\n", name_);
                    return SPIN_TIMEOUT;
                }
            }
            
            /* reset if message has timed out */
            if (mode_ != MODE_FIRST_FF) {
                if (c_time > last_msg_timeout_time) {
                    // 一帧消息接收超时
                    printf("NodeHandle %s: msg timeout\n", name_);
                    mode_ = MODE_FIRST_FF;
                }
            }
            
            int data = hardware_.read();
            if (data < 0) {
                // 没有新的数据
                break;
            }
            checksum_ += data;
            
            if (mode_ == MODE_FIRST_FF) {
                if (data == 0xff) {
                    mode_++;
                    // 记录当前时间, 以计算一帧消息的超时时间
                    last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
                } else {
                    //                    printf("? %d\n", data);
                }
            } else if (mode_ == MODE_PROTOCOL_VER) {
                if (data == PROTOCOL_VER) {
                    mode_++;
                } else {
                    // 错误的协议版本, 发送同步时间请求以告知上位机协议版本
                    mode_ = MODE_FIRST_FF;
                    printf("NodeHandle %s: bad PROTOCOL_VER\n", name_);
                    if (configured_ == false)
                        requestSyncTime(); /* send a msg back showing our protocol version */
                }
            } else if (mode_ == MODE_SIZE_L) /* bottom half of message size */
            {
                bytes_ = data;
                index_ = 0;
                mode_++;
                checksum_ = data; /* first byte for calculating size checksum */
            } else if (mode_ == MODE_SIZE_H) /* top half of message size */
            {
                bytes_ += data << 8;
                mode_++;
            } else if (mode_ == MODE_SIZE_CHECKSUM) {
//                if (bytes_ > INPUT_SIZE) {
//                    mode_ = MODE_FIRST_FF; /* Abandon the frame if the msg len is wrong */
//                    printf("NodeHandle: msg too long: %d > %d\n", bytes_, INPUT_SIZE);
//                } else
                if ((checksum_ % 256) == 255) {
                    if (message_in_ == NULL || bytes_ > message_in_size_) {
                        mode_ = MODE_FIRST_FF; /* Abandon the frame if the msg len is wrong */
                        printf("NodeHandle %s: msg too long: %d/%d\n", name_, bytes_, message_in_size_);
                    } else {
                        mode_++;
                    }
//                    message_in_ = (uint8_t*) malloc(bytes_);
//                    if (message_in_ != NULL) {
//                        mode_++;
//                    } else {
//                        mode_ = MODE_FIRST_FF; /* Abandon the frame if the msg len is wrong */
//                        printf("NodeHandle: msg too long: %d\n", bytes_);
//                    }
                    //                    printf("size: %d\n", bytes_);
                } else {
                    mode_ = MODE_FIRST_FF; /* Abandon the frame if the msg len is wrong */
                    printf("NodeHandle %s: bad size checksum\n", name_);
                }
            } else if (mode_ == MODE_TOPIC_L) /* bottom half of topic id */
            {
                topic_ = data;
                mode_++;
                checksum_ = data; /* first byte included in checksum */
            } else if (mode_ == MODE_TOPIC_H) /* top half of topic id */
            {
                topic_ += data << 8;
                mode_ = MODE_MESSAGE;
                if (bytes_ == 0) {
                    mode_ = MODE_MSG_CHECKSUM;
                    //                    printf("recved\n");
                }
            } else if (mode_ == MODE_MESSAGE) /* message data being recieved */
            {
                message_in_[index_++] = (uint8_t)data;
                bytes_--;
                /* is message complete? if so, checksum */
                if (bytes_ == 0) {
                    mode_ = MODE_MSG_CHECKSUM;
                }
            } else if (mode_ == MODE_MSG_CHECKSUM) /* do checksum */
            {
                mode_ = MODE_FIRST_FF;
                if ((checksum_ % 256) == 255) {
                    // 消息数据校验正确
                    if (topic_ == TopicInfo::ID_PUBLISHER) {
                        // 上位机请求下位机开始通信
                        requestSyncTime();
                        if (negotiateTopics() == 0) {
                            printf("NodeHandle %s: negotiate topics\n", name_);
                            if (!configured_) {
                                configured_ = true;
                                configured_t_ = hardware_.time();
                                printf("NodeHandle %s: Connected to PC\n", name_);
                            }
                        }
                        last_sync_time = c_time;
                        last_sync_receive_time = c_time;
                        return SPIN_ERR;
                    } else if (topic_ == TopicInfo::ID_TIME) {
                        // 接收到了时间同步响应
                        syncTime(message_in_);
                    } else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST) {
                        // 接收到了参数响应
                        req_param_resp.deserialize(message_in_);
                        param_recieved = true;
                    } else if (topic_ == TopicInfo::ID_TX_STOP) {
                        // 接收到了上位机结束消息
                        configured_ = false;
                        printf("NodeHandle %s: Disconnected\n", name_);
                    } else {
                        // 接收到了其他消息
                        if (subscribers[topic_ - 100])
                            subscribers[topic_ - 100]->callback(message_in_);
                    }
                } else {
                    printf("NodeHandle %s: bad data checksum\n", name_);
                }
//                if (message_in_ != NULL) {
//                    free(message_in_);
//                    message_in_ = NULL;
//                }
            }
        }
        
        /* occasionally sync time */
        if (configured_ && c_time - last_sync_time >= SYNC_INTERVAL) {
            // 在连接后, 每隔SYNC_INTERVAL同步一次时间
            requestSyncTime();
            last_sync_time = c_time;
        }
        
        return SPIN_OK;
    }
    
    /* Are we connected to the PC? */
    virtual bool connected() {
        return configured_;
    };
    
    /********************************************************************
     * Time functions
     */
    
    // 请求同步时间
    int requestSyncTime() {
        int ret;
        std_msgs::Time t;
        ret = publish(TopicInfo::ID_TIME, &t);
        rt_time = hardware_.time_nsec();
        // printf("requestSyncTime()\n");
        return ret;
    }
    
    // 处理时间同步响应
    void syncTime(uint8_t* data) {
        uint64_t hardware_nsec = hardware_.time_nsec();
        // Round Trip Time, 同步时间的往返时间, ns
        uint64_t RTT_nsec = hardware_nsec - rt_time;
        // 当前时间
        uint64_t old_nsec = hardware_nsec + nsec_offset;
        
        std_msgs::Time t;
        t.deserialize(data);
        // 上位机返回的时间, 应加上RTT, 才为当前时间
        uint64_t new_nsec = t.data.sec * uint64_t(1000000000) + t.data.nsec + RTT_nsec;
        
        // 上位机与下位机的时间差, 下位机时间 + 时间差 = 上位机时间
        int64_t time_offset_ns = (int64_t) (new_nsec - old_nsec);
        
        char buf[256];
        
        if (time_variance <= 0) {
            // 首次进行时间同步
            
            // 当前时间直接设为上位机时间
            addTimeOffset(time_offset_ns);
            // 当前时间的方差直接设为RTT的平方
            time_variance = (RTT_nsec / 1e9f / 2) * (RTT_nsec / 1e9f / 2);
            
            // 输出时间同步状态
            snprintf(buf, 256,
                     "SYNC_TIME: RTT=%" PRIu32 " OFFSET=%" PRIu32 ".%09" PRIu32 "",
                     RTT_nsec / 1000, t.data.sec, t.data.nsec);
            loginfo(buf);
        } else {
            // use Kalman Filter to smooth time
            // 使用卡尔曼滤波器进行平滑的时间同步
            
            // variance of recved time is (RTT / 2)^2
            // 本次时间测量的方差, 为RTT的平方
            float R = (RTT_nsec / 1e9f / 2) * (RTT_nsec / 1e9f / 2);
            // time drift (30 PPM)
            // 时间偏移率, 估计为 30 PPM (30 per million)
            float Q = (hardware_.time() - last_sync_receive_time) / 1000.0f * (0.00003f * 0.00003f);
            // variance minus
            // 估计当前的时间方差
            float P = time_variance + Q;
            // Kalman Gain
            // 计算卡尔曼增益
            float K = P / (P + R);
            // new variance
            // 计算本次时间校准后的方差
            time_variance = (1 - K) * P;
            // apply Kalman Gain
            // 应用卡尔曼增益, 更新当前时间
            int64_t time_adjust_ns = (int32_t) (time_offset_ns * K);
            
            addTimeOffset(time_adjust_ns);
            
            if (timer_print_sync_.is_timeout()) {
                snprintf(buf, 256,
//                         "%u.%09u RTT=%d OFFSET=%+d STD=%d ADJ=%+d",
                         "\tRTT=% 5d OFFSET=% +5d STD=% 3d ADJ=% +3d",
//                         (unsigned int) now().sec, (unsigned int) now().nsec,
                         int(RTT_nsec / 1000), int(time_offset_ns / 1000),
                         int(std::sqrt(time_variance) * 1000000), int(time_adjust_ns / 1000));
                loginfo(buf);
            }
            //                printf(buf);
            //                printf("\n");
        }
        
        last_sync_receive_time = hardware_.time();
    }
    
    Time now() {
        uint64_t ns = hardware_.time_nsec();
        return fromNSec(ns);
    }
    
    Time fromNSec(uint64_t ns) {
        Time current_time;
        ns += nsec_offset;
        current_time.sec = (uint32_t) (ns / 1000000000);
        current_time.nsec = (uint32_t) (ns % 1000000000);
        return current_time;
    }
    
    void setNow(Time& new_now) {
        uint64_t ns = hardware_.time_nsec();
        setTimeOffset(ns);
    }
    
    // 设定时间偏移量, 单位ns
    void setTimeOffset(uint64_t offset) {
        auto pri = taskENTER_CRITICAL_FROM_ISR();
        nsec_offset = offset;
        taskEXIT_CRITICAL_FROM_ISR(pri);
    }
    
    // 增量改变时间偏移量, 单位ns
    void addTimeOffset(int64_t offset) {
        auto pri = taskENTER_CRITICAL_FROM_ISR();
        nsec_offset += offset;
        taskEXIT_CRITICAL_FROM_ISR(pri);
    }
    
    /********************************************************************
     * Topic Management
     */
    
    /* Register a new publisher */
    bool advertise(Publisher& p) {
        for (int i = 0; i < MAX_PUBLISHERS; i++) {
            if (publishers[i] == 0) // empty slot
            {
                publishers[i] = &p;
                p.id_ = i + 100 + MAX_SUBSCRIBERS;
                p.nh_ = this;
                return true;
            }
        }
        return false;
    }
    
    /* Register a new subscriber */
    template<typename SubscriberT>
    bool subscribe(SubscriberT& s) {
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (subscribers[i] == 0) // empty slot
            {
                subscribers[i] = static_cast<Subscriber_*>(&s);
                s.id_ = i + 100;
                return true;
            }
        }
        return false;
    }
    
    /* Register a new Service Server */
    template<typename MReq, typename MRes, typename ObjT>
    bool advertiseService(ServiceServer<MReq, MRes, ObjT>& srv) {
        bool v = advertise(srv.pub);
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (subscribers[i] == 0) // empty slot
            {
                subscribers[i] = static_cast<Subscriber_*>(&srv);
                srv.id_ = i + 100;
                return v;
            }
        }
        return false;
    }
    
    /* Register a new Service Client */
    template<typename MReq, typename MRes>
    bool serviceClient(ServiceClient<MReq, MRes>& srv) {
        bool v = advertise(srv.pub);
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (subscribers[i] == 0) // empty slot
            {
                subscribers[i] = static_cast<Subscriber_*>(&srv);
                srv.id_ = i + 100;
                return v;
            }
        }
        return false;
    }
    
    int negotiateTopics() {
        rosserial_msgs::TopicInfo ti;
        int i;
        for (i = 0; i < MAX_PUBLISHERS; i++) {
            if (publishers[i] != 0) // non-empty slot
            {
                ti.topic_id = publishers[i]->id_;
                ti.topic_name = (char*) publishers[i]->topic_;
                ti.message_type = (char*) publishers[i]->msg_->getType();
                ti.md5sum = (char*) publishers[i]->msg_->getMD5();
                ti.buffer_size = message_out_size_;
                int ret = publish(publishers[i]->getEndpointType(), &ti);
                if (ret < 0) {
                    return ret;
                }
            }
        }
        for (i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (subscribers[i] != 0) // non-empty slot
            {
                ti.topic_id = subscribers[i]->id_;
                ti.topic_name = (char*) subscribers[i]->topic_;
                ti.message_type = (char*) subscribers[i]->getMsgType();
                ti.md5sum = (char*) subscribers[i]->getMsgMD5();
                ti.buffer_size = message_in_size_;
                int ret = publish(subscribers[i]->getEndpointType(), &ti);
                if (ret < 0) {
                    return ret;
                }
            }
        }
        return 0;
    }
    
    virtual int publish(int id, const Msg* msg) {
        if (id >= 100 && !configured_)
            return 0;
        if (hardware_.time() - configured_t_ <= 20)
            // delay 20ms to send msg
            return 0;
        
        uint8_t* message_out = message_out_;
//        std::unique_ptr<uint8_t[]> message_out = std::make_unique<uint8_t[]>(OUTPUT_SIZE);
        
        /* serialize message */
        int l = msg->serialize(message_out + 7);
        
        /* setup the header */
        message_out[0] = 0xff;
        message_out[1] = PROTOCOL_VER;
        message_out[2] = (uint8_t) ((uint16_t) l & 255);
        message_out[3] = (uint8_t) ((uint16_t) l >> 8);
        message_out[4] = 255 - ((message_out[2] + message_out[3]) % 256);
        message_out[5] = (uint8_t) ((int16_t) id & 255);
        message_out[6] = (uint8_t) ((int16_t) id >> 8);
        
        /* calculate checksum */
        int chk = 0;
        for (int i = 5; i < l + 7; i++)
            chk += message_out[i];
        l += 7;
        message_out[l++] = 255 - (chk % 256);
        
        int ret;
        if (l > message_out_size_) {
            printf(
                "NodeHandle %s: Message from device dropped: message larger than buffer.\n", name_);
            return -1;
        } else {
            ret = hardware_.write(message_out, l);
            if (ret != l) {
                printf(
                    "NodeHandle %s: Message from device dropped: write(l=%d) failed ret=%d.\n", name_, l, ret);
                if (ret < 0) {
                    // 认为断开连接
                    configured_ = false;
                    printf("NodeHandle %s: Lost (cannot send)\n", name_);
                    hardware_.init();
                }
                return -2;
            } else {
                return 0;
            }
        }
    }
    
    /********************************************************************
     * Logging
     */

private:
    void log(char byte, const char* msg) {
        rosserial_msgs::Log l;
        l.level = byte;
        l.msg = (char*) msg;
        publish(rosserial_msgs::TopicInfo::ID_LOG, &l);
    }

public:
    void logdebug(const char* msg) {
        log(rosserial_msgs::Log::ROSDEBUG, msg);
    }
    void loginfo(const char* msg) {
        log(rosserial_msgs::Log::INFO, msg);
    }
    void logwarn(const char* msg) {
        log(rosserial_msgs::Log::WARN, msg);
    }
    void logerror(const char* msg) {
        log(rosserial_msgs::Log::ERROR, msg);
    }
    void logfatal(const char* msg) {
        log(rosserial_msgs::Log::FATAL, msg);
    }
    
    /********************************************************************
     * Parameters
     */

private:
    bool param_recieved;
    rosserial_msgs::RequestParamResponse req_param_resp;
    
    bool requestParam(const char* name, int time_out = 1000) {
        param_recieved = false;
        rosserial_msgs::RequestParamRequest req;
        req.name = (char*) name;
        publish(TopicInfo::ID_PARAMETER_REQUEST, &req);
        uint32_t end_time = hardware_.time() + time_out;
        while (!param_recieved) {
            spinOnce();
            if (hardware_.time() > end_time) {
                logwarn("Failed to get param: timeout expired");
                return false;
            }
        }
        return true;
    }

public:
    bool getParam(const char* name, int* param, int length = 1, int timeout = 1000) {
        if (requestParam(name, timeout)) {
            if (length == req_param_resp.ints_length) {
                //copy it over
                for (int i = 0; i < length; i++)
                    param[i] = req_param_resp.ints[i];
                return true;
            } else {
                logwarn("Failed to get param: length mismatch");
            }
        }
        return false;
    }
    bool getParam(const char* name, float* param, int length = 1, int timeout = 1000) {
        if (requestParam(name, timeout)) {
            if (length == req_param_resp.floats_length) {
                //copy it over
                for (int i = 0; i < length; i++)
                    param[i] = req_param_resp.floats[i];
                return true;
            } else {
                logwarn("Failed to get param: length mismatch");
            }
        }
        return false;
    }
    bool getParam(const char* name, char** param, int length = 1, int timeout = 1000) {
        if (requestParam(name, timeout)) {
            if (length == req_param_resp.strings_length) {
                //copy it over
                for (int i = 0; i < length; i++)
                    strcpy(param[i], req_param_resp.strings[i]);
                return true;
            } else {
                logwarn("Failed to get param: length mismatch");
            }
        }
        return false;
    }
};

} // namespace ros

#endif