//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_DEBUG_TERMINAL_H
#define ARM_ROBOT_F427_V3_0_DEBUG_TERMINAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"

void debug_socket_init();
int debug_socket_write(void* data, size_t len);
int debug_socket_read(void* data, size_t len);

#ifdef __cplusplus
};
#endif

#endif //ARM_ROBOT_F427_V3_0_DEBUG_TERMINAL_H
