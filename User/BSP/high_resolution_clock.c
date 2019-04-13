/*
 * high_resolution_clock.c
 *
 *  Created on: 2018年6月4日
 *      Author: shuixiang
 */

#include "high_resolution_clock.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

volatile uint64_t MY_DWT_LAST_CYCLE_COUNT = 0;

#ifdef __cplusplus
}
#endif /* __cplusplus */

