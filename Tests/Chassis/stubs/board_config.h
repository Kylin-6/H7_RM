#pragma once
#include "fdcan.h"
struct BoardHardware { FDCAN_HandleTypeDef *chassis_wheel_bus; FDCAN_HandleTypeDef *chassis_steer_bus; };
const BoardHardware &BoardConfig_Get(void);
