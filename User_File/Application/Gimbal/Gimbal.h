#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** ControlTask 的 1 kHz 入口；状态发布为 100 Hz。 */
void Gimbal_Update(void);

#if LEGACY_INFANTRY

#include "SpeedPlanning.h"
#include "dmmotor.h"

/* ===== 老步兵云台：单轴 DM 电机 MIT 速度控制（移植自 rm/demo 的 APP/GimbalTask.c）===== */

/** 云台偏航 DM 电机的节点 ID 与主控接收 ID，与 demo 的 bsp_def.h 一致。 */
#define GIMBAL_YAW_MOTOR_CAN_ID (0x03U)
#define GIMBAL_YAW_MOTOR_MASTER_ID (0x05U)

typedef struct
{
    Class_DMMotor Yaw_Motor;               ///< 偏航轴 DM 电机（MIT 模式）
    SpeedPlanningState Yaw_Speed_Planning; ///< 偏航速度规划状态
    float Yaw_Speed_Command;               ///< 本周期下发的偏航速度，rad/s
    float Yaw_Mit_Kd;                      ///< 当前 MIT 阻尼增益
    float Yaw_Mit_Torque_Feedforward;      ///< 当前 MIT 力矩前馈，N·m
} DMGimbal_t;

/** 初始化云台 DM 电机与速度规划；上电默认保持失能。 */
void Gimbal_Init(void);
/** 执行一次云台速度闭环并向电机下发 MIT 命令。 */
void Gimbal_Loop(void);

extern DMGimbal_t Gimbal;

#elif GIMBAL

#include "Gimbal_Config.h"
#include "alg_pid.h"
#include "dmmotor.h"

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE,
    Gimbal_Status_ENABLING,
    Gimbal_Status_READY,
    Gimbal_Status_FAULT,
    Gimbal_Status_CONFIG_ERROR,
};

struct Struct_Gimbal
{
    Class_DMMotor Yaw_Motor;
    Class_DMMotor Pitch_Motor;
    Class_PID Yaw_Angle_PID;
    Class_PID Yaw_Speed_PID;
    Enum_Gimbal_Status status = Gimbal_Status_DISABLE;
    float Target_Yaw_Angle = 0.0f;
    float Target_Pitch_Angle = 0.0f;
    float Target_Yaw_Speed = 0.0f;
    float Target_Pitch_Speed = 0.0f;
};

extern Struct_Gimbal Gimbal;
/** 仅启动阶段调用一次；复制配置并注册驱动，不等待应答、不置零、不切换电机模式。 */
bool Gimbal_Init(const Struct_Gimbal_Config &config = Gimbal_Default_Config());
#endif
#endif
