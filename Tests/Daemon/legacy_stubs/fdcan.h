#pragma once

/* 主机替身：FDCAN_HandleTypeDef 由 bsp_can.h 替身统一定义。 */
#include "bsp_can.h"

extern "C" uint32_t HAL_GetTick(void);
