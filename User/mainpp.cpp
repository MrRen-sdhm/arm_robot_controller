//
// Created by shuixiang on 2018/9/28.
//
#include <iostream>

extern "C" {
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
}

#include "debug_terminal.h"

#include "robot.hpp"

static std::unique_ptr<Robot> robot;

// 注册HAL层回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    DMABuffer_UART::on_uart_error(huart);
}
extern "C" void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* huart) {
    DMABuffer_UART::on_uart_rx_abort_complete(huart);
}
extern "C" void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef* huart) {
    fputs("abt tx?\n", stderr);
    //    DMABuffer_UART::on_uart_rx_abort_complete(huart);
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    DMABuffer_UART::on_rx_dma_complete(huart);
}
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    DMABuffer_UART::on_tx_dma_complete(huart);
}
//extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
//    if (hi2c == bmx055_camera.hi2c) {
//        bmx055_camera.on_i2c_dma_complete();
//    }
//}
//extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
//    if (hi2c == bmx055_camera.hi2c) {
//        bmx055_camera.on_i2c_dma_complete();
//    }
//}
extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
#ifndef DISABLE_LIFT
    if (!robot)
        return;
    if (htim == robot->lift_.motor_.htim)
        robot->lift_.motor_.on_tim_oc_elapsed();
#endif
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    fputs("CAN RxFifo0MsgPending\n", stdout);
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    fputs("CAN RxFifo1MsgPending\n", stdout);
}
extern "C" void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    fputs("CAN RxFifo0Full\n", stdout);
}
extern "C" void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    fputs("CAN RxFifo1Full\n", stdout);
}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    printf("CAN Error 0x%08X\n", (unsigned int)hcan->ErrorCode);
}

// 被 StartDefaultTask() 调用
extern "C" int mainpp() {
    // 启用看门狗定时器, 超时: 1s
    MX_IWDG_Init();
    
    // 初始化 UDP 调试输出
    debug_socket_init();
    
    std::cout << "System start" << std::endl;
    
    robot.reset(new(robot_mem_region) Robot("HUSTAC-PG"));
    robot->init();

    for (;;) {
        // 喂狗
        HAL_IWDG_Refresh(&hiwdg);
        robot->spin_once();
    }
}
