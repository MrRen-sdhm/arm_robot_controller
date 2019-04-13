//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP
#define ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP

#include <cstring>

#include "lwip/sockets.h"
#include "mutex.hpp"

#include "pipe.hpp"

namespace hustac {

class PipeUDPBroadcast : public Pipe {
public:
    PipeUDPBroadcast(const char *bind_ip, uint16_t recv_port, uint16_t send_port);
    bool connected() override { return sock_in_fd_ >= 0 && sock_out_fd_ >= 0; }
    int readsome(void *, size_t) override;
    int write(void *, size_t) override;
    ~PipeUDPBroadcast() override;
protected:
    void _reconnect();
    void _spin_once();
protected:
    struct sockaddr_in bind_addr_;
    struct sockaddr_in send_addr_;
    int sock_in_fd_ = -1;
    int sock_out_fd_ = -1;
};

inline PipeUDPBroadcast::PipeUDPBroadcast(const char *bind_ip, uint16_t recv_port, uint16_t send_port) {
    memset(&bind_addr_, 0, sizeof(bind_addr_));
    bind_addr_.sin_family = AF_INET;
    bind_addr_.sin_addr.s_addr = inet_addr(bind_ip);
    bind_addr_.sin_port = htons(recv_port);
    memset(&send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_addr.s_addr = inet_addr("255.255.255.255");
    send_addr_.sin_port = htons(send_port);
    _spin_once();
}

inline PipeUDPBroadcast::~PipeUDPBroadcast() {
    if (sock_in_fd_ >= 0)
        lwip_close(sock_in_fd_);
    sock_in_fd_ = -1;
    if (sock_out_fd_ >= 0)
        lwip_close(sock_out_fd_);
    sock_out_fd_ = -1;
}

inline void PipeUDPBroadcast::_reconnect() {
    if (sock_in_fd_ >= 0)
        lwip_close(sock_in_fd_);
    sock_in_fd_ = -1;
    if (sock_out_fd_ >= 0)
        lwip_close(sock_out_fd_);
    sock_out_fd_ = -1;
    _spin_once();
}

inline void PipeUDPBroadcast::_spin_once() {
    if (sock_in_fd_ < 0) {
        int sock_fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd < 0) {
            perror("PipeUDPBroadcast: Can not create socket");
            return;
        }
        
        // enable nonblocking
        int flags;
        flags = lwip_fcntl(sock_fd, F_GETFL, 0);
        if (flags < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_GETFL) failed");
            lwip_close(sock_fd);
            return;
        }
        flags |= O_NONBLOCK;
        if (lwip_fcntl(sock_fd, F_SETFL, flags) < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_SETFL, O_NONBLOCK) failed");
            lwip_close(sock_fd);
            return;
        }
        
        if (lwip_bind(sock_fd, (sockaddr *) &bind_addr_, (socklen_t) sizeof(bind_addr_)) < 0) {
            perror("PipeUDPBroadcast: lwip_bind() failed");
            lwip_close(sock_fd);
            return;
        }
        
        sock_in_fd_ = sock_fd;
    }
    if (sock_out_fd_ < 0) {
        int sock_fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd < 0) {
            perror("PipeUDPBroadcast: Can not create socket");
            return;
        }
        
        // enable broadcast
        int optval = 1;
        if (lwip_setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval)) < 0) {
            perror("PipeUDPBroadcast: lwip_setsockopt(SO_BROADCAST, 1) failed");
            lwip_close(sock_fd);
            return;
        }
        
        // enable nonblocking
        int flags;
        flags = lwip_fcntl(sock_fd, F_GETFL, 0);
        if (flags < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_GETFL) failed");
            lwip_close(sock_fd);
            return;
        }
        flags |= O_NONBLOCK;
        if (lwip_fcntl(sock_fd, F_SETFL, flags) < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_SETFL, O_NONBLOCK) failed");
            lwip_close(sock_fd);
            return;
        }
        
        if (lwip_connect(sock_fd, (sockaddr *) &send_addr_, (socklen_t) sizeof(send_addr_)) < 0) {
            perror("PipeUDPBroadcast: lwip_connect() failed");
            lwip_close(sock_fd);
            return;
        }
    
        sock_out_fd_ = sock_fd;
    }
}

inline int PipeUDPBroadcast::readsome(void *data, size_t len) {
    if (sock_in_fd_ >= 0) {
        int ret = lwip_read(sock_in_fd_, data, len);
        if (ret >= 0) {
            // 成功读取
            return ret;
        } else {
            // 读取失败, 重新连接
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("PipeUDPBroadcast: lwip_read() failed");
                _reconnect();
                return -2;
            }
            return -1;
        }
    } else {
        _spin_once();
        return -3;
    }
}

inline int PipeUDPBroadcast::write(void *data, size_t len) {
    if (sock_out_fd_ >= 0) {
        int ret = lwip_write(sock_out_fd_, data, (size_t) len);
        if (ret == len) {
            // 成功写入
            return len;
        } else if (ret >= 0) {
            // 仅成功写入部分
            fprintf(stderr, "NodeHandler: lwip_write() %d/%d\n", ret, len);
            return ret;
        } else {
            // 写入失败, 重新连接
            perror("NodeHandler: lwip_write() failed");
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                _reconnect();
                return -2;
            } else {
                return 0;
            }
        }
    } else {
        _spin_once();
        return -3;
    }
}

}

#endif //ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP
