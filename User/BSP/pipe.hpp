//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_PIPE_HPP
#define ARM_ROBOT_F427_V3_0_PIPE_HPP

namespace hustac {

class Pipe {
public:
    // 调用构造函数, 将立即进行连接
    // 当中途连接失败或发生其他错误时, 将自动重连
    // 调用析构函数, 将断开连接
    virtual bool connected() = 0;
    virtual int readsome(void *, size_t) = 0;
    virtual int write(void *, size_t) = 0;
    virtual ~Pipe() {}
};

}

#endif //ARM_ROBOT_F427_V3_0_PIPE_HPP
