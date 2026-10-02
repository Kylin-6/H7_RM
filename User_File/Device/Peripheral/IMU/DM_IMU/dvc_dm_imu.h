/**
 * ******************************************************************************
 * @file    dvc_dm_imu.h
 * @brief   达妙 DM-IMU-L1 姿态传感器驱动（Classic CAN）。
 * @details 按官方说明书 V1.3 附录 C 的 CAN 数据协议解码：
 *          - 欧拉角帧（DATA[0]=0x03）：DATA[2..3]=俯仰 [-90,+90) 度、
 *            DATA[4..5]=偏航 [-180,+180) 度、DATA[6..7]=横滚 [-180,+180) 度，
 *            均为小端 uint16，DATA[1] 保留；
 *          - 角速度帧（DATA[0]=0x02）：DATA[2..7]=X/Y/Z 角速度，
 *            uint16 线性还原，物理范围 ±34.88 rad/s。
 *          请求帧与数据帧使用两个固定 ID，接收方向经框架 CAN BSP 的回调注册。
 *          Pitch 解码路径与老步兵云台板原工程逐字节一致。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 * ******************************************************************************
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

/** 欧拉角数据帧寄存器号（说明书附录 C 表 B-1）。 */
#define DM_IMU_REG_EULER (0x03U)
/** 角速度数据帧寄存器号（说明书附录 C 表 B-1）。 */
#define DM_IMU_REG_GYRO (0x02U)

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
 * @brief 请求一帧欧拉角数据（寄存器 0x03）。
 * @return true 表示请求帧已入发送队列，不代表传感器已应答。
 * @note 由控制周期调用，本周期使用最近收到的数据帧。
 */
bool DM_IMU_RequestEuler(void);

/**
 * @brief 请求一帧角速度数据（寄存器 0x02）。
 * @return true 表示请求帧已入发送队列，不代表传感器已应答。
 * @note 与欧拉角请求共用同一请求帧格式；需要角速度时与欧拉角请求交替发出。
 */
bool DM_IMU_RequestGyro(void);

/**
 * @brief 读取最近一次收到的 Pitch 角。
 * @param pitch_deg 输出角度，单位度。
 * @return true 表示至少收到过一帧合法欧拉角数据。
 */
bool DM_IMU_GetPitch(float *pitch_deg);

typedef struct
{
    float pitch_deg;
    float yaw_deg;
    float roll_deg;
    uint32_t sequence;
    uint32_t timestamp_ms;
    bool valid;
} Struct_DM_IMU_Euler_Snapshot;
/** 欧拉角、序号、接收时间来自同一帧；短临界区内复制，不在控制周期重试。 */
Struct_DM_IMU_Euler_Snapshot DM_IMU_GetEulerSnapshot(void);

/**
 * @brief 连同接收序号一起读取 Pitch 角。
 * @param pitch_deg 输出角度，单位度。
 * @param sequence 每收到一帧合法欧拉角数据自增一次。
 * @return true 表示至少收到过一帧合法欧拉角数据。
 * @note 角度与序号在同一段重试临界读取中取得，可用于差分求角速度。
 */
bool DM_IMU_GetPitchSample(float *pitch_deg, uint32_t *sequence);

/**
 * @brief 读取最近一帧欧拉角三轴数据。
 * @param pitch_deg 输出俯仰角，单位度，范围 [-90, +90)。
 * @param yaw_deg   输出偏航角，单位度，范围 [-180, +180)。
 * @param roll_deg  输出横滚角，单位度，范围 [-180, +180)。
 * @return true 表示至少收到过一帧合法欧拉角数据。
 * @note 俯仰解码与 DM_IMU_GetPitch 同源；偏航/横滚来自同一帧的其余字段。
 */
bool DM_IMU_GetEuler(float *pitch_deg, float *yaw_deg, float *roll_deg);

/**
 * @brief 读取最近一帧三轴角速度。
 * @param x_rad_s 输出 X 轴角速度，rad/s。
 * @param y_rad_s 输出 Y 轴角速度，rad/s。
 * @param z_rad_s 输出 Z 轴角速度，rad/s。
 * @return true 表示至少收到过一帧合法角速度数据。
 * @note 传感器坐标系与整机安装方向的关系须由板级配置确认。
 */
bool DM_IMU_GetGyro(float *x_rad_s, float *y_rad_s, float *z_rad_s);

/**
 * @brief 查询最近一次收到任意合法数据帧的时间。
 * @return HAL tick 毫秒值；从未收到时为 0。
 */
uint32_t DM_IMU_GetLastRxMs(void);

#ifdef __cplusplus
}
#endif

#endif /* DVC_DM_IMU_H */
