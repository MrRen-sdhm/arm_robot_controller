#include <array>
#include <cstring>

#include "usart.h"

#include "dmabuffer_uart.hpp"

namespace hustac {

std::list<DMABuffer_UART*> DMABuffer_UART::instances;

void DMABuffer_UART::init_uart(const char* name) {
    for (DMABuffer_UART* dmabuffer : instances) {
        if (!name || strcmp(dmabuffer->name_, name) == 0) {
            dmabuffer->_init();
        }
    }
}

DMABuffer_UART* DMABuffer_UART::get_uart(const char* name) {
    for (DMABuffer_UART* dmabuffer : instances) {
        if (strcmp(dmabuffer->name_, name) == 0) {
            return dmabuffer;
        }
    }
    return NULL;
}
void DMABuffer_UART::on_uart_rx_abort_complete(UART_HandleTypeDef* huart) {
    for (DMABuffer_UART* dmabuffer : instances) {
        dmabuffer->_on_uart_rx_abort_complete(huart);
    }
}
void DMABuffer_UART::on_uart_error(UART_HandleTypeDef* huart) {
    for (DMABuffer_UART* dmabuffer : instances) {
        dmabuffer->_on_uart_error(huart);
    }
}
void DMABuffer_UART::on_rx_dma_complete(UART_HandleTypeDef* huart) {
    for (DMABuffer_UART* dmabuffer : instances) {
        dmabuffer->_on_rx_dma_complete(huart);
    }
}
void DMABuffer_UART::on_tx_dma_complete(UART_HandleTypeDef* huart) {
    for (DMABuffer_UART* dmabuffer : instances) {
        dmabuffer->_on_tx_dma_complete(huart);
    }
}

#define define_full_duplex_with_static_buffer(name, huart, tx_len, rx_len) \
    uint8_t name##_TX_BUF[tx_len];                           \
    uint8_t name##_RX_BUF[rx_len];                           \
    DMABuffer_UART name(#name, huart, name##_TX_BUF, tx_len, name##_RX_BUF, rx_len);

#define define_half_duplex_with_static_buffer(name, huart, write_enable, tx_len, rx_len) \
    uint8_t name##_TX_BUF[tx_len];                                          \
    uint8_t name##_RX_BUF[rx_len];                                          \
    DMABuffer_UART name(#name, huart, write_enable, name##_TX_BUF, tx_len, name##_RX_BUF, rx_len);

// 注意缓冲区尺寸必须为2的整数倍
//define_full_duplex_with_static_buffer(UART_ROSSERIAL, &huart5, 4096, 1024); // 全双工TTL: 2M 8N1
//define_full_duplex_with_static_buffer(UART_TERMINAL, &huart3, 2048, 128); // 全双工TTL: 2M 8N1
//define_half_duplex_with_static_buffer(UART_HEAD, &huart6, NULL, 128, 128); // 半双工单线TTL: 1M 8N1
//define_half_duplex_with_static_buffer(RS485_ARM_LEFT, &huart2, &RS485_1_WE, 128, 128); // 半双工RS485(带有输出使能引脚): 1M 8N1
//define_half_duplex_with_static_buffer(RS485_ARM_RIGHT, &huart1, &RS485_2_WE, 128, 128); // 半双工RS485(带有输出使能引脚): 1M 8N1
//define_half_duplex_with_static_buffer(RS485_HAND, &huart4, &RS485_3_WE, 128, 128); // 半双工RS485(带有输出使能引脚): 1M 8N1

#undef define_full_duplex_with_static_buffer
#undef define_half_duplex_with_static_buffer
}
