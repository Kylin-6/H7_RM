/**
 * ******************************************************************************
 * @file    dm_imu_ins.h
 * @brief   DM-IMU → INS_State_Topic 桥：把 DM-IMU 数据发布为框架统一姿态 Topic。
 * @details 框架 Gimbal 等应用只消费 `INS_State_Topic`（默认由 BMI088 链路经
 *          sys_imu 发布）。本桥让 DM-IMU 成为该 Topic 的另一个来源：任务上下文
 *          周期调用 Update()，交替请求欧拉角/角速度帧并发布；Daemon 活性判断与
 *          sys_imu 一致。同一 Topic 仍只允许一个发布者，调用方必须保证 BMI088
 *          InsTask 与本桥二选一。
 * @note    传感器坐标系与整机 INS 轴向的映射须经板级核对；当前按原始 XYZ 透传。
 * ******************************************************************************
 */
#ifndef DM_IMU_INS_H
#define DM_IMU_INS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化 INS 桥：注册 Daemon 活性监控。
 * @return true 表示 Daemon 注册成功；失败时调用方不得启动发布。
 * @note 只能调用一次，须在 DM_IMU_Init 之后调用。
 */
bool DM_IMU_InsBridge_Init(void);

/**
 * @brief 任务上下文 1 kHz 周期入口：请求一帧数据并发布 INS 状态。
 * @note 奇偶周期交替请求欧拉角（0x03）与角速度（0x02）帧；每周期都把最近
 *       收到的欧拉角 + 角速度合成为 INS_State 发布。数据不新鲜时发零姿态。
 */
void DM_IMU_InsBridge_Update(void);

/** 最近 100 ms 内是否收到过合法数据帧（与设备离线判定一致）。 */
bool DM_IMU_InsBridge_IsFresh(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_IMU_INS_H */
