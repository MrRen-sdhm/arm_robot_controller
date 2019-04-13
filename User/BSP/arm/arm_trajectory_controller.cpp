#include <functional>

#include "arm_trajectory_controller.hpp"

// 本文件中的变量置于IRAM2, 无法DMA访问

namespace hustac {

__attribute__((section(".ccmram-arm-controller"))) uint8_t ARM_RIGHT_CTRL_MEM_REGION[sizeof(ArmControllerT)];
__attribute__((section(".ccmram-arm-controller"))) uint8_t ARM_LEFT_CTRL_MEM_REGION[sizeof(ArmControllerT)];
    
}
