//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_PIPE_TCP_SERVER_HPP
#define ARM_ROBOT_F427_V3_0_PIPE_TCP_SERVER_HPP

#include "pipe.hpp"

namespace hustac {

class PipeTCPServer : public Pipe {
public:
    PipeTCPServer(const char *bind_ip, uint16_t bind_port);
    bool connected() override { return !client_fds_.empty(); }
    int readsome(void *, size_t) override;
    int write(void *, size_t) override;
protected:
    struct sockaddr_in bind_addr_;
    int server_fd_ = -1;
    std::vector<int> client_fds_;
    Mutex mutex_;
};

}

#endif //ARM_ROBOT_F427_V3_0_PIPE_TCP_SERVER_HPP
