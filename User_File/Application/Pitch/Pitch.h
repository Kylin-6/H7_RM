/**
 * @file Pitch.h
 * @brief Pitch 轴应用（老步兵云台板配置）：DM 电机 MIT 力矩控制 + DM-IMU 角度反馈。
 * @details
 * 移植自老步兵云台板工程（H7_RM）的 `Application/Pitch`，控制律逐项保持不变：
 *
 * - 角度反馈使用 DM-IMU 的 Pitch 欧拉角，不再使用电机单圈编码器；
 * - 位置环输出力矩，叠加遥控目标速度前馈、IMU 角速度阻尼、连续 Stribeck 摩擦
 *   补偿和低带宽扰动估计，最后经 DM MIT 的 `t_ff` 下发（电机侧刚度/阻尼为 0）；
 * - 遥控目标走「限最大斜率」路径，手工 / 上层目标走在线限加加速度 S 曲线路径。
 *
 * 与云台板原实现的差异只有两处，均为分层调整，不改变数值：
 * 1. 通道 → 目标角的两级低通与线性映射移到 `Communication`（输入适配），本模块
 *    接收已经滤波映射后的目标角；
 * 2. 2 s 上电使能延迟改为「每次使能前重新计时」，以便链路中断后能安全重新使能。
 *
 * @note 与云台板一致，Pitch 电机方向与 DM-IMU 的 Pitch 正方向相反，力矩取负下发。
 * @author Kylin-6（原始实现）/ H7_BSP 移植
 */
#ifndef PITCH_H
#define PITCH_H

#include "dmmotor.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Pitch DM 电机控制帧 ID（Class_DMMotor 的 can_id）。 */
#define PITCH_MOTOR_ID          (0x009U)
/** Pitch DM 电机反馈帧 ID（Class_DMMotor 的 master_id）。 */
#define PITCH_MOTOR_FEEDBACK_ID (0x019U)
/** 反馈解码范围，须与电机固件参数一致，与云台板既有驱动数值保持一致。 */
#define PITCH_MOTOR_P_MAX_RAD   (3.14f)
#define PITCH_MOTOR_V_MAX_RAD_S (30.0f)
#define PITCH_MOTOR_T_MAX_NM    (10.0f)

/** DM-IMU Pitch 目标角度下限，单位 rad（-40 度）。 */
#define PITCH_TARGET_MIN_RAD    (-0.6981317f)
/** DM-IMU Pitch 目标角度上限，单位 rad（+15 度）。 */
#define PITCH_TARGET_MAX_RAD    (0.2617994f)
/** MIT 力矩指令限幅，单位 N·m。 */
#define PITCH_MAX_TORQUE_NM     (0.5f)
/** Stribeck 模型正方向的静摩擦力矩，单位 N·m。 */
#define PITCH_POSITIVE_STATIC_FRICTION_TORQUE_NM  (0.038f)
/** Stribeck 模型负方向的静摩擦力矩，单位 N·m。 */
#define PITCH_NEGATIVE_STATIC_FRICTION_TORQUE_NM  (0.040f)
/** Stribeck 模型正方向的库仑摩擦力矩，单位 N·m。 */
#define PITCH_POSITIVE_COULOMB_FRICTION_TORQUE_NM (0.015f)
/** Stribeck 模型负方向的库仑摩擦力矩，单位 N·m。 */
#define PITCH_NEGATIVE_COULOMB_FRICTION_TORQUE_NM (0.020f)
/** 静摩擦向库仑摩擦过渡的特征速度，单位 rad/s。 */
#define PITCH_STRIBECK_VELOCITY_RAD_S    (0.100f)
/** 将位置误差转换为零速摩擦补偿方向的系数，单位 1/s。 */
#define PITCH_STRIBECK_ERROR_GAIN        (3.0f)
/** 摩擦方向 tanh 平滑速度，单位 rad/s。 */
#define PITCH_STRIBECK_DIRECTION_SMOOTH_RAD_S (0.080f)
/** 电机反馈速度不参与控制。 */
#define PITCH_VELOCITY_DAMPING          (0.0f)
/** 遥控目标正方向速度前馈系数。 */
#define PITCH_POSITIVE_VELOCITY_FEEDFORWARD (0.012f)
/** 遥控目标负方向速度前馈系数。 */
#define PITCH_NEGATIVE_VELOCITY_FEEDFORWARD (0.018f)
/** 遥控目标最大变化速度，单位 rad/s。 */
#define PITCH_REMOTE_TARGET_VELOCITY_MAX_RAD_S (3.0f)
/** S 曲线轨迹最大速度，单位 rad/s。 */
#define PITCH_TRAJECTORY_MAX_VELOCITY_RAD_S (1.0f)
/** S 曲线轨迹最大加速度，单位 rad/s^2。 */
#define PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2   (3.0f)
/** S 曲线轨迹最大加加速度，单位 rad/s^3。 */
#define PITCH_TRAJECTORY_MAX_JERK_RAD_S3    (15.0f)
/** 电机反馈速度低通时间常数，单位 s。 */
#define PITCH_VELOCITY_FILTER_TAU_S     (0.020f)
/** IMU 角速度估计低通时间常数，单位 s。 */
#define PITCH_IMU_VELOCITY_FILTER_TAU_S (0.010f)
/** IMU 角速度阻尼系数，单位 N·m·s/rad。 */
#define PITCH_IMU_VELOCITY_DAMPING      (0.043f)
/** IMU 差分角速度限幅，防止异常帧产生力矩冲击，单位 rad/s。 */
#define PITCH_IMU_VELOCITY_MAX_RAD_S    (3.0f)
/** 低带宽扰动估计积分增益，单位 N·m/(rad·s)。 */
#define PITCH_DISTURBANCE_INTEGRAL_GAIN       (0.40f)
/** 扰动力矩补偿限幅，单位 N·m。 */
#define PITCH_DISTURBANCE_TORQUE_MAX_NM       (0.030f)
/** 目标低于该速度时允许学习扰动，单位 rad/s。 */
#define PITCH_DISTURBANCE_TARGET_SPEED_RAD_S  (0.10f)
/** 实际低于该速度时允许学习扰动，单位 rad/s。 */
#define PITCH_DISTURBANCE_ACTUAL_SPEED_RAD_S  (0.10f)
/** 运动时旧扰动补偿衰减到零的时间常数，单位 s。 */
#define PITCH_DISTURBANCE_DECAY_TAU_S          (0.20f)
/** 每次使能前等待 DM-IMU 数据并延迟下发使能帧的时间，单位 ms。 */
#define PITCH_ENABLE_DELAY_MS   (2000U)

/**
 * @brief 初始化 Pitch 电机、DM-IMU 与位置环参数。
 * @return true 表示电机反馈回调、DM-IMU 回调均注册成功。
 * @note 可重复调用，只有第一次生效；需要 BSP_CAN_ConfigInit 已执行。
 *       使能帧不在这里发送，而是在 Pitch_Update 中延迟 PITCH_ENABLE_DELAY_MS。
 */
bool Pitch_Init(void);

/**
 * @brief 执行一次 Pitch 控制计算并向电机下发 MIT 力矩。
 * @param requested_target_rad 上层目标角度，单位 rad，未限幅。
 * @param target_valid true 表示本周期存在有效上层目标（遥控通道或命令）。
 * @param enabled false 表示失能：立即下发失能帧并停止 MIT 输出。
 * @note 由 1 kHz 控制任务周期调用。目标无效、DM-IMU 无数据或尚未使能时不下发指令。
 */
void Pitch_Update(float requested_target_rad, bool target_valid, bool enabled);

/**
 * @brief 设置手工 / 上层目标角度，设置后走 S 曲线轨迹路径并忽略上层目标。
 * @param angle_rad 目标角度，单位 rad，入口即限幅。
 * @note 与云台板原实现一致，供调试 / 上位机使用。
 */
void Pitch_SetTargetAngle(float angle_rad);

/**
 * @brief 清除手工目标，恢复跟随上层目标。
 */
void Pitch_ClearTargetAngle(void);

/**
 * @brief 获取轨迹规划后的目标角度。
 * @return 目标角度，单位 rad。
 */
float Pitch_GetTargetAngle(void);

/**
 * @brief 获取 DM-IMU 最近一次 Pitch 角度。
 * @param pitch_deg 输出角度，单位度。
 * @return true 表示已收到有效欧拉角数据。
 */
bool Pitch_GetImuPitchDeg(float *pitch_deg);

/**
 * @brief 获取最近一次 MIT 最终力矩输出。
 * @return PID、摩擦前馈、阻尼与扰动补偿之和，单位 N·m。
 */
float Pitch_GetOutputTorque(void);

/**
 * @brief 获取低带宽扰动估计力矩。
 * @return 当前扰动补偿，单位 N·m。
 */
float Pitch_GetDisturbanceTorque(void);

/**
 * @brief 获取 Pitch 电机当前位置。
 * @return 当前单圈位置，单位 rad。
 */
float Pitch_GetPositionRad(void);

/**
 * @brief 获取 Pitch 电机当前原始编码器值。
 * @return 原始编码器值，0~65535。
 */
uint16_t Pitch_GetEncoder(void);

/**
 * @brief 查询电机是否已完成使能。
 * @return true 表示已下发使能帧且反馈在线。
 */
bool Pitch_IsEnabled(void);

/**
 * @brief 查询 DM-IMU 是否有过有效数据。
 * @return true 表示至少收到过一帧合法欧拉角。
 */
bool Pitch_IsImuValid(void);

/**
 * @brief 获取 DM-IMU 差分并滤波后的 Pitch 角速度。
 * @return 角速度，单位 rad/s；用于阻尼补偿与遥测。
 */
float Pitch_GetImuVelocityRadS(void);

/**
 * @brief 获取 Pitch 电机实例。
 * @return 电机实例指针。
 */
Class_DMMotor *Pitch_GetMotor(void);

#ifdef __cplusplus
}
#endif

#endif /* PITCH_H */
