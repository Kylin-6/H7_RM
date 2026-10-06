#ifndef GIMBAL_IMU_TRANSPORT_H
#define GIMBAL_IMU_TRANSPORT_H

#include "bsp_can.h"

constexpr uint32_t GIMBAL_IMU_MAX_AGE_US = 100000U;
/** ControlTask 启动时调用一次；板间总线由 BoardConfig 绑定。 */
bool GimbalImuTransport_Init(void);
/** 1 kHz ControlTask 调用：云台 2 ms 提交新样本，底盘配对并发布独立 Topic。 */
void GimbalImuTransport_Update(void);

#endif
