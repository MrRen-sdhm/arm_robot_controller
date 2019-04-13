/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
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

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include <cstring>
#include <cstdio>
#include <cerrno>
#include <vector>
#include <stdexcept>
#include <mutex>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"

#include <lwip/sockets.h>
#include "cmsis_os.h"

#include "usart.h"

#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"

using namespace hustac;

class STM32Hardware {
public:
    STM32Hardware()
        : tcp_port_(0) {
    }
    
    void init() {
    }
    
    void init(const char* port_name) {
        tcp_port_ = std::atoi(port_name);
        if (pipe_ == NULL)
            pipe_ = new(pipe_mem_) PipeTCPClient("192.168.123.2", tcp_port_);
        else
            pipe_->reconnect();
    }
    
    int read() {
        if (pipe_ == NULL)
            init();
        uint8_t ch;
        int ret = pipe_->readsome(&ch, 1);
        if (ret == 1) {
            return ch;
        } else {
            return -1;
        }
    }
    
    int write(uint8_t* data, int len) {
        if (pipe_ == NULL)
            init();
        return pipe_->write(data, len);
    }
    
    uint32_t time() {
        return osKernelSysTick();
    }
    
    uint64_t time_nsec() {
        return MY_GetNanoSecFromCycle(MY_GetCycleCount());
    }
    
    ~STM32Hardware() {
        if (pipe_ != NULL) {
            delete pipe_;
            pipe_ = NULL;
        }
    }

public:
    int tcp_port_;
    PipeTCPClient* pipe_ = NULL;
    uint8_t pipe_mem_[sizeof(PipeTCPClient)];
};

#endif

