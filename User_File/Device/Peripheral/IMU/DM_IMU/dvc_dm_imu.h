/**
 ******************************************************************************
 * @file    dvc_dm_imu.h
 * @brief   达妙 DM-IMU-L1 姿态传感器驱动（Classic CAN）。
 * @details 老步兵云台板的 Pitch 轴反馈来源：接受者只解析欧拉角帧的第 2、3 字节
 *          （小端 uint16），按 [-90, +90) 度 / 180 度跨度解码为 Pitch 角度。
 *          请求帧与数据帧使用两个固定 ID，接收方向经框架 CAN BSP 的回调注册。
 * @note    移植自老步兵云台板工程（H7_RM）Device/Peripheral/IMU/DM_IMU，
 *          解码公式、寄存器号与请求帧内容逐字节保持不变。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 ******************************************************************************
 */
#ifndef DVC_DM_IMU_H
#define DVC_DM_IMU_H

#include "bsp_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** 云台板现有 DM-IMU 使用的请求 / 命令 ID。 */
#define DM_IMU_DEFAULT_CAN_ID (0x66U)
/** 云台板现有 DM-IMU 使用的数据 / 应答 ID。 */
#define DM_IMU_DEFAULT_MST_ID (0x33U)

/**
 * @brief 把一路 DM-IMU 绑定到 CAN 总线并注册接收回调。
 * @param hfdcan 传感器所在总线句柄，非空。
 * @param can_id 请求 / 命令帧 ID。
 * @param mst_id 数据 / 应答帧 ID。
 * @return true 表示接收回调注册成功。
 * @note 可重复调用，成功注册后再次注册同一 (总线, ID) 会被 BSP 拒绝。
 */
bool DM_IMU_Init(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, uint32_t mst_id);

/**
 * @brief 请求一帧欧拉角数据。
 * @return true 表示请求帧已入发送队列，不代表传感器已应答。
 * @note 由控制周期调用，本周期使用最近收到的数据帧。
 */
bool DM_IMU_RequestEuler(void);

/**
 * @brief 读取最近一次收到的 Pitch 角。
 * @param pitch_deg 输出角度，单位度。
 * @return true 表示至少收到过一帧合法欧拉角数据。
 */
bool DM_IMU_GetPitch(float *pitch_deg);

/**
 * @brief 连同接收序号一起读取 Pitch 角。
 * @param pitch_deg 输出角度，单位度。
 * @param sequence 每收到一帧合法欧拉角数据自增一次。
 * @return true 表示至少收到过一帧合法欧拉角数据。
 * @note 角度与序号在同一段重试临界读取中取得，可用于差分求角速度。
 */
bool DM_IMU_GetPitchSample(float *pitch_deg, uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif /* DVC_DM_IMU_H */
