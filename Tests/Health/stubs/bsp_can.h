#pragma once
#include "../../Boundary/stubs/bsp_can.h"
inline void __set_PRIMASK(uint32_t state) { test_irq_mask = state; }
