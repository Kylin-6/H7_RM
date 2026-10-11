/**
 * @file dm_imu_ins.cpp
 * @brief 老步兵 DM-IMU 姿态适配：设备快照 → SI → 唯一 INS Topic。
 * @details Pitch 阻尼沿用欧拉角差分测速（限幅 3 rad/s、tau=10 ms），
 *          不替换为尚未上板标定的原生 gyro。每个新欧拉角帧只发布一次；
 *          无应答时不刷新 Topic 时间戳，Gimbal 按 100 ms 时效失能。
 */
#include "dm_imu_ins.h"
#include "dvc_dm_imu.h"
#include "board_config.h"
#include "message_center.h"
#include "stm32h7xx_hal.h"

namespace
{
constexpr float kDegToRad = 0.0174532925f;
constexpr uint32_t kInsTimeoutMs = 100U;
constexpr float kVelocityTauS = 0.010f;
constexpr float kVelocityMaxRadS = 3.0f;
bool initialized;
bool has_sample;
uint32_t last_sequence;
float last_pitch_rad;
float velocity_rad_s;
} // namespace

extern "C" bool DM_IMU_InsBridge_Init(void)
{
    if (initialized)
    {
        return true;
    }
    initialized = DM_IMU_Init(BoardConfig_Get().external_imu_bus,
                              DM_IMU_DEFAULT_CAN_ID, DM_IMU_DEFAULT_MST_ID);
    return initialized;
}

extern "C" bool DM_IMU_InsBridge_IsFresh(void)
{
    /* 这是姿态数据 freshness（INS 是否还能用于控制）；传感器链路 liveness
     * 唯一见 DM_IMU_IsOnline() 的 Daemon 结果，两者独立维护。 */
    const auto sample = DM_IMU_GetEulerSnapshot();
    return initialized && sample.valid &&
           HAL_GetTick() - sample.timestamp_ms <= kInsTimeoutMs;
}

extern "C" void DM_IMU_InsBridge_Update(void)
{
    if (!initialized)
    {
        return;
    }
    // 单次提交失败在下一周期重试；不阻塞、不循环等待应答。
    (void) DM_IMU_RequestEuler();
    const auto sample = DM_IMU_GetEulerSnapshot();
    if (!sample.valid || HAL_GetTick() - sample.timestamp_ms > kInsTimeoutMs)
    {
        has_sample = false;
        velocity_rad_s = 0.0f;
        return;
    }
    if (has_sample && sample.sequence == last_sequence)
    {
        return;
    }
    const float pitch_rad = sample.pitch_deg * kDegToRad;
    if (has_sample)
    {
        // 保留原 Pitch 的采样约定：每个合法欧拉角序号代表标称 1 ms。
        const float dt_s = 0.001f * static_cast<float>(sample.sequence - last_sequence);
        float velocity = (pitch_rad - last_pitch_rad) / dt_s;
        if (velocity > kVelocityMaxRadS)
        {
            velocity = kVelocityMaxRadS;
        }
        if (velocity < -kVelocityMaxRadS)
        {
            velocity = -kVelocityMaxRadS;
        }
        velocity_rad_s += dt_s / (kVelocityTauS + dt_s) * (velocity - velocity_rad_s);
    }
    last_pitch_rad = pitch_rad;
    last_sequence = sample.sequence;
    has_sample = true;
    INS_State ins{};
    ins.pitch_rad = pitch_rad;
    ins.yaw_rad = sample.yaw_deg * kDegToRad;
    ins.roll_rad = sample.roll_deg * kDegToRad;
    // 统一 INS 的 Pitch 角速度槽为 Y；这是姿态差分速度，不是原始机体系 gyro。
    ins.gyro_y_rad_s = velocity_rad_s;
    MessageCenter::INS_State_Topic.Publish(ins);
}
