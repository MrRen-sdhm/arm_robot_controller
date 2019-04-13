//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_PIPE_TCP_CLIENT_HPP
#define ARM_ROBOT_F427_V3_0_PIPE_TCP_CLIENT_HPP

#include <cstring>

#include "lwip/sockets.h"
#include "mutex.hpp"

#include "pipe.hpp"

namespace hustac {

class PipeTCPClient : public Pipe {
public:
    PipeTCPClient(const char *server_ip, uint16_t server_port, uint32_t connect_timeout = 1000, uint32_t retry_timeout = 1000);
    void reconnect();
    bool connected() override { return sock_fd_ >= 0 && !connecting_; }
    int readsome(void *, size_t) override;
    int write(void *, size_t) override;
    ~PipeTCPClient() override;
protected:
    void _spin_once();
protected:
    struct sockaddr_in server_addr_;
    int sock_fd_ = -1;
    bool connecting_ = false;
    uint32_t connect_timeout_;
    uint32_t retry_timeout_;
    uint32_t last_connect_ = 0;
    Mutex mutex_;
};

inline PipeTCPClient::PipeTCPClient(const char *server_ip, uint16_t server_port, uint32_t connect_timeout, uint32_t retry_timeout)
    : connect_timeout_(connect_timeout), retry_timeout_(retry_timeout) {
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = inet_addr(server_ip);
    server_addr_.sin_port = htons(server_port);
    _spin_once();
}

inline PipeTCPClient::~PipeTCPClient() {
    if (sock_fd_ >= 0)
        lwip_close(sock_fd_);
    sock_fd_ = -1;
}

inline void PipeTCPClient::reconnect() {
    std::unique_lock<Mutex> lock(mutex_);
    if (sock_fd_ >= 0)
        lwip_close(sock_fd_);
    sock_fd_ = -1;
    connecting_ = false;
    lock.unlock();
    _spin_once();
}

inline void PipeTCPClient::_spin_once() {
    std::unique_lock<Mutex> lock(mutex_);
    int ret;
    
    if (sock_fd_ < 0) {
        // create socket
        uint32_t current = osKernelSysTick();
        if (int32_t(current - (last_connect_ + retry_timeout_)) < 0)
            // waiting
            return;
        last_connect_ = current;
        sock_fd_ = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_fd_ < 0) {
            perror("PipeTCPClient: Can not create socket");
            goto error_exit;
        }
        
        // set to nonblocking
        int flags;
        flags = lwip_fcntl(sock_fd_, F_GETFL, 0);
        if (flags < 0) {
            fputs("PipeTCPClient: lwip_fcntl(F_GETFL) failed\n", stderr);
            goto error_exit;
        }
        flags |= O_NONBLOCK;
        if (lwip_fcntl(sock_fd_, F_SETFL, flags) < 0) {
            fputs("PipeTCPClient: lwip_fcntl(F_SETFL) failed\n", stderr);
            goto error_exit;
        }
        
        // set TCP no delay
        flags = 1;
        if (lwip_setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, (void*)&flags, sizeof(flags)) < 0) {
            fputs("PipeTCPClient: lwip_setsockopt(TCP_NODELAY) failed\n", stderr);
            goto error_exit;
        }
        
        // connect
        ret = lwip_connect(sock_fd_, (struct sockaddr *) &server_addr_, sizeof(struct sockaddr));
        if (ret == 0) {
            fputs("PipeTCPClient: Connected without wait\n", stdout);
            connecting_ = false;
        } else if (ret < 0 && errno == EINPROGRESS) {
//            fputs("PipeTCPClient: Connecting\n", stdout);
            connecting_ = true;
        } else {
            perror("PipeTCPClient: connect() failed");
            goto error_exit;
        }
    } else if (connecting_) {
        // connecting
        fd_set readset;
        fd_set writeset;
        fd_set errset;
        FD_ZERO(&readset);
        FD_SET(sock_fd_, &readset);
        FD_ZERO(&writeset);
        FD_SET(sock_fd_, &writeset);
        FD_ZERO(&errset);
        FD_SET(sock_fd_, &errset);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        // select without waiting
        ret = lwip_select(sock_fd_ + 1, &readset, &writeset, &errset, &tv);
        if (ret < 0) {
            perror("PipeTCPClient: select() failed");
            goto error_exit;
        } else if (ret > 0) {
            if (FD_ISSET(sock_fd_, &writeset) && !FD_ISSET(sock_fd_, &readset) && !FD_ISSET(sock_fd_, &errset)) {
                // connected
                fputs("PipeTCPClient: Connected (writeset)\n", stderr);
                connecting_ = false;
            } else if (FD_ISSET(sock_fd_, &readset) || FD_ISSET(sock_fd_, &errset)) {
                // connect failed
                int err = 0;
                socklen_t errlen = sizeof(err);
                if (lwip_getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, &err, &errlen) < 0) {
                    perror("PipeTCPClient: lwip_getsockopt() failed");
                }
                if (err != ECONNRESET) {
                    fprintf(stderr, "PipeTCPClient: Connect error: %s\n", strerror(err));
                }
                goto error_exit;
            } else {
                // unkonwn
                fputs("PipeTCPClient: select() unknown\n", stderr);
                goto error_exit;
            }
        } else {
            // connecting
            if (int(osKernelSysTick() - last_connect_) >= connect_timeout_) {
                lock.unlock();
//                fprintf(stderr, "PipeTCPClient: Connect timeout\n");
                reconnect();
            }
        }
    } else {
        // connected
    }
    return;
    error_exit:
    if (sock_fd_ >= 0) lwip_close(sock_fd_);
    sock_fd_ = -1;
    connecting_ = false;
}

inline int PipeTCPClient::readsome(void *data, size_t len) {
    std::unique_lock<Mutex> lock(mutex_);
    if (connected()) {
        uint8_t ch;
        int ret = lwip_read(sock_fd_, data, len);
        if (ret >= 0) {
            // 成功读取
            return ret;
        } else {
            // 读取失败, 重新连接
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("PipeTCPClient: read() failed");
                lock.unlock();
                reconnect();
                return -2;
            }
            return -1;
        }
    } else {
        lock.unlock();
        _spin_once();
        return -3;
    }
}

inline int PipeTCPClient::write(void *data, size_t len) {
    std::unique_lock<Mutex> lock(mutex_);
    if (connected()) {
        int ret = lwip_write(sock_fd_, data, (size_t) len);
        if (ret == len) {
            // 成功写入
            return len;
        } else if (ret >= 0) {
            // 仅成功写入部分
            fprintf(stderr, "PipeTCPClient: write() %d/%d\n", ret, len);
            return ret;
        } else {
            // 写入失败, 重新连接
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("PipeTCPClient: write() failed");
                lock.unlock();
                reconnect();
                return -2;
            } else {
                return 0;
            }
        }
    } else {
        lock.unlock();
        _spin_once();
        return -3;
    }
}

}
#endif //ARM_ROBOT_F427_V3_0_PIPE_TCP_CLIENT_HPP
