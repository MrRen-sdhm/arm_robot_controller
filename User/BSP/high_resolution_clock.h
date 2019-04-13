/*
 * high_resolution_clock.h
 *
 *  Created on: 2018年6月4日
 *      Author: shuixiang
 */

#ifndef BSP_HIGH_RESOLUTION_CLOCK_H_
#define BSP_HIGH_RESOLUTION_CLOCK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// 通过 SysTick 获取开机到现在的CPU时钟数
// 注意: 使用该函数需要确保 SysTick 中断为使能状态
static uint64_t MY_GetCycleCount(void) {
    uint32_t tick_0, tick_1;
    uint32_t count;
    // 重复读取计数器值, 直到确保 uwTick 与 SysTick->VAL 是同步的
    do {
        tick_0 = osKernelSysTick();
        count = SysTick->VAL;
        tick_1 = osKernelSysTick();
    } while (tick_0 != tick_1);
    // SysTick 的周期为 SysTick->LOAD + 1, SysTick 为倒计时
    return (uint64_t)tick_0 * (SysTick->LOAD + 1) + (SysTick->LOAD - count);
}

/* 通过 DWT 获取时间戳 */
// 使能 DWT 定时器
static void MY_DWTTimerInit(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // volatile uint32_t *DWT_LAR  = (uint32_t *) 0xE0001FB0;
    // *DWT_LAR = 0xC5ACCE55;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

extern volatile uint64_t MY_DWT_LAST_CYCLE_COUNT;

// 通过 DWT 获取开机到现在的CPU时钟数
// 注意: 每2^32个时钟周期(72M 频率下约 59.65秒), 至少应调用该函数一次
static uint64_t MY_DWTGetCycleCount(void) {
    // 使用 J-Link 下载程序后, 可能会自动禁用 trace, 需要保证为使能状态
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 记录中断使能状态, 并暂停中断
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // 计算当前CPU周期数
    MY_DWT_LAST_CYCLE_COUNT += DWT->CYCCNT - (uint32_t)(MY_DWT_LAST_CYCLE_COUNT);
    // 恢复中断
    __set_PRIMASK(primask);
    return MY_DWT_LAST_CYCLE_COUNT;
}

static uint64_t MY_GetNanoSecFromCycle(uint64_t cpu_cycle_count) {
    uint32_t sec = cpu_cycle_count / SystemCoreClock;
    uint32_t nsec = (cpu_cycle_count % SystemCoreClock) * 1000000000 / SystemCoreClock;
    return (uint64_t)sec * 1000000000 + nsec;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BSP_HIGH_RESOLUTION_CLOCK_H_ */
