#pragma once

#include "stm32h7xx.h"

/* 中断屏蔽等 intrinsics 由各测试目标的 stm32h7xx.h 替身提供；
 * 生产代码在设备层使用的 HAL_GetTick 在主机测试中由测试文件按微秒时间实现。 */
extern "C" uint32_t HAL_GetTick(void);
