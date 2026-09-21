#pragma once
#include <stdint.h>
uint32_t osKernelGetTickCount();
uint32_t osKernelGetTickFreq();
int32_t osDelayUntil(uint32_t ticks);
