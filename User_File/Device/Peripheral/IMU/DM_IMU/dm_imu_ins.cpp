/**
 * ******************************************************************************
 * @file    dm_imu_ins.cpp
 * @brief   DM-IMU → INS_State_Topic 桥实现。
 * @details 发布模式与 System/IMU/sys_imu（BMI088 链路）对齐：Daemon 30 ms 活性、
 *          Publisher 固定绑定 INS_State_Topic、只在任务上下文发布。单位换算：
 *          欧拉角 度 → rad，角速度帧本身是 rad/s。数据不新鲜时发布零姿态并置
 *          ins_valid=false，由消费端（如框架 Gimbal 的 10 ms 新鲜度检查）决定
 *          安全行为。
 * ******************************************************************************
 */

#include "dm_imu_ins.h"

#include "dvc_dm_imu.h"

#include "daemon.h"
#include "message_center.h"
#include "message_types.h"

#include "stm32h7xx_hal.h"

#include <cmath>
#include <stddef.h>

namespace
{
/** 姿态角 度 → rad。 */
constexpr float kDegToRad = 0.017453292519943295f;
/** INS 活性判定窗口；与 sys_imu 的 30 ms Daemon 语义一致。 */
constexpr uint32_t kInsTimeoutMs = 100U;
/** 请求周期计数：奇偶交替请求欧拉角 / 角速度帧。 */
uint32_t request_divider;
bool bridge_initialized;
bool bridge_registered;
} // namespace

extern "C" bool DM_IMU_InsBridge_Init(void)
{
    if (bridge_initialized)
    {
        return true;
    }

    Daemon ins_daemon{30U};
    bridge_registered = DaemonManager::Register(ins_daemon);
    request_divider = 0U;
    bridge_initialized = true;
    return bridge_registered;
}

extern "C" bool DM_IMU_InsBridge_IsFresh(void)
{
    return DM_IMU_GetLastRxMs() != 0U &&
           (HAL_GetTick() - DM_IMU_GetLastRxMs()) <= kInsTimeoutMs;
}

extern "C" void DM_IMU_InsBridge_Update(void)
{
    if (!bridge_initialized || !bridge_registered)
    {
        return;
    }

    /* 奇偶交替请求：单帧应答率不变的前提下同时覆盖欧拉角与角速度。 */
    ++request_divider;
    if ((request_divider & 1U) != 0U)
    {
        (void)DM_IMU_RequestEuler();
    }
    else
    {
        (void)DM_IMU_RequestGyro();
    }

    const bool fresh = DM_IMU_InsBridge_IsFresh();
    INS_State ins{};
    if (fresh)
    {
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
        float roll_deg = 0.0f;
        if (DM_IMU_GetEuler(&pitch_deg, &yaw_deg, &roll_deg))
        {
            ins.pitch_rad = pitch_deg * kDegToRad;
            ins.yaw_rad = yaw_deg * kDegToRad;
            ins.roll_rad = roll_deg * kDegToRad;
        }
        float gyro_x = 0.0f;
        float gyro_y = 0.0f;
        float gyro_z = 0.0f;
        if (DM_IMU_GetGyro(&gyro_x, &gyro_y, &gyro_z))
        {
            ins.gyro_x_rad_s = gyro_x;
            ins.gyro_y_rad_s = gyro_y;
            ins.gyro_z_rad_s = gyro_z;
        }
    }
    /* fresh=false 时保持零姿态默认值发布，消费端按新鲜度判定安全行为。 */
    MessageCenter::INS_State_Topic.Publish(ins);
}
