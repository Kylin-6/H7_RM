#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** ControlTask 的 1 kHz 入口；状态发布为 100 Hz。 */
void Gimbal_Update(void);

#if GIMBAL

#if LEGACY_INFANTRY_GIMBAL

#include "QD4310.h"
#include "alg_pid.h"
#include "alg_fsm.h"
#include "bsp_can.h"

#define YAW_ID 0
#define PITCH_ID 1
#define GIMBAL_PITCH_MIN_ANGLE_RAD 5.85f
#define GIMBAL_PITCH_MAX_ANGLE_RAD 6.25f

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0, ///< 尚未初始化或已禁用
    Gimbal_Status_READY,       ///< 两轴电机均已就绪
    Gimbal_Status_YAW_ERROR,   ///< Yaw 轴使能或通信异常
    Gimbal_Status_PITCH_ERROR, ///< Pitch 轴使能或通信异常
};

/**
 * @brief 老步兵云台板的云台状态。
 *
 * 云台板只有 Pitch 轴参与实际控制：DM 电机（MIT）+ DM-IMU 角度反馈，实现在
 * `Application/Pitch`，本模块只负责模式门控与反馈汇总。Yaw 轴沿用云台板原工程的
 * QD4310 + INS 实现，但云台板当前 BMI088 硬件故障、原工程也没有调度
 * `Gimbal_Init` / `Gimbal_Loop`，因此默认关闭；需要 Yaw 时打开 CMake 选项
 * `LEGACY_INFANTRY_GIMBAL_YAW`。
 */
typedef struct
{
    Class_FSM<5> Gimbal_FSM; ///< Yaw 轴初始化状态机（Yaw 关闭时不使用）
    QD4310_t Yaw_Motor;      ///< 偏航轴电机（Yaw 关闭时不初始化）

    float Target_Yaw_Angle;  ///< Yaw 目标角，rad
    float Target_Yaw_Speed;  ///< Yaw 目标角速度，rad/s

    Class_PID Yaw_Angle_PID; ///< Yaw 角度外环
    Class_PID Yaw_Speed_PID; ///< Yaw 速度内环
} LegacyGimbal_t;

extern LegacyGimbal_t Gimbal;

/** 初始化云台设备和控制器；电机异常时在有限重试后返回。 */
void Gimbal_Init(void);
/** 执行一次已经通过状态守卫的云台闭环计算。 */
void Gimbal_Loop(void);

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
