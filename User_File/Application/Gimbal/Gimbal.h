#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** ControlTask 的 1 kHz 入口；状态发布为 100 Hz。 */
void Gimbal_Update(void);

#if GIMBAL

#if LEGACY_INFANTRY_GIMBAL

/**
 * 老步兵云台板（legacy 过渡路径）：初始化 Pitch 轴设备与控制律
 * （DM 电机 MIT + DM-IMU，实现在 Application/Pitch）。
 * 框架路径（PitchOnly + ImuTorque）完成上板验证后本分支与
 * Application/Pitch 一并退役，见 Gimbal/README.md。
 */
void Gimbal_Init(void);

#else

#include "Gimbal_Config.h"

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE,
    Gimbal_Status_ENABLING,
    Gimbal_Status_READY,
    Gimbal_Status_FAULT,
    Gimbal_Status_CONFIG_ERROR,
};

/** 仅启动阶段调用一次；复制配置并注册驱动，不等待应答、不置零、不切换电机模式。 */
bool Gimbal_Init(const Struct_Gimbal_Config &config = Gimbal_Default_Config());
/** 返回应用状态快照；仅供任务上下文读取，不允许外部改写状态或直接控制电机。 */
Enum_Gimbal_Status Gimbal_GetStatus(void);
#endif /* LEGACY_INFANTRY_GIMBAL */

#endif /* GIMBAL */
#endif
