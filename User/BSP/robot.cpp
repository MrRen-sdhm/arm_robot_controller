#include "robot.hpp"

namespace hustac {

uint8_t robot_mem_region[sizeof(Robot)];
__attribute__((section(".ccmram-nh-cpp-buf-out"))) uint8_t NH_CPP_BUF_OUT[NH_CPP_BUF_OUT_SIZE];
__attribute__((section(".ccmram-nh-cpp-buf-in"))) uint8_t NH_CPP_BUF_IN[NH_CPP_BUF_IN_SIZE];
__attribute__((section(".ccmram-nh-py-buf-out"))) uint8_t NH_PY_BUF_OUT[NH_PY_BUF_OUT_SIZE];
__attribute__((section(".ccmram-nh-py-buf-in"))) uint8_t NH_PY_BUF_IN[NH_PY_BUF_IN_SIZE];

}