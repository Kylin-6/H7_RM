/**
 ******************************************************************************
 * @file    dvc_dm_imu.cpp
 * @brief   达妙 DM-IMU-L1 驱动实现。
 * @details 接收回调运行在 FDCAN 中断中，只做寄存器号校验与定点解码；请求发送走
 *          CAN BSP 的插入队列，与控制周期解耦。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 ******************************************************************************
 */

#include "dvc_dm_imu.h"

#include "stm32h7xx_hal.h"

#include <stddef.h>

namespace
{
/** 欧拉角寄存器号；数据帧首字节必须等于该值。 */
constexpr uint8_t kEulerRegister = 0x03U;
/** 角速度寄存器号；数据帧首字节必须等于该值。 */
constexpr uint8_t kGyroRegister = 0x02U;
/** Pitch 解码下限，单位度。 */
constexpr float kPitchMinDeg = -90.0f;
/** Pitch 解码跨度，单位度。 */
constexpr float kPitchSpanDeg = 180.0f;
/** 偏航/横滚解码下限，单位度（说明书表 B-2）。 */
constexpr float kYawRollMinDeg = -180.0f;
/** 偏航/横滚解码跨度，单位度。 */
constexpr float kYawRollSpanDeg = 360.0f;
/** 角速度解码下限，rad/s（说明书表 B-2）。 */
constexpr float kGyroMinRadS = -34.88f;
/** 角速度解码跨度，rad/s。 */
constexpr float kGyroSpanRadS = 69.76f;
/** uint16 满量程，用于归一化解码。 */
constexpr float kUint16Max = 65535.0f;
/** 超过该时间没有收到新姿态即判定 DM-IMU 离线。 */
constexpr uint32_t kPitchTimeoutMs = 100U;

FDCAN_HandleTypeDef *dm_imu_can;
uint32_t dm_imu_can_id;
volatile float dm_imu_pitch_deg;
volatile bool dm_imu_pitch_valid;
volatile uint32_t dm_imu_pitch_sequence;
volatile uint32_t dm_imu_last_rx_ms;
/* 偏航/横滚（欧拉角帧附带字段）与三轴角速度（0x02 帧）。 */
volatile float dm_imu_yaw_deg;
volatile float dm_imu_roll_deg;
volatile float dm_imu_gyro_rad_s[3];
volatile bool dm_imu_gyro_valid;
volatile uint32_t dm_imu_gyro_sequence;

/** 把 uint16 原始值按线性量程还原为物理解码值。 */
float DecodeUint16(uint16_t value, float minimum, float span)
{
    return static_cast<float>(value) * span / kUint16Max + minimum;
}

/**
 * @brief 解析一帧欧拉角数据；寄存器号不符时直接丢弃。
 * @note 中断上下文，不做浮点格式化、不阻塞。
 */
void DM_IMU_RxCallback(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id,
                       uint8_t *data,
                       uint32_t len,
                       void *context)
{
    (void)hfdcan;
    (void)id;
    (void)context;

    if (data == nullptr || len < 8U)
    {
        return;
    }

    if (data[0] == kEulerRegister)
    {
        /* 说明书附录 C 表 B-1：DATA[2..3]=俯仰、DATA[4..5]=偏航、DATA[6..7]=横滚，
         * 小端 uint16，DATA[1] 保留。Pitch 解码与原工程逐字节一致。 */
        const uint16_t pitch_raw = static_cast<uint16_t>(data[2]) |
                                   (static_cast<uint16_t>(data[3]) << 8U);
        const uint16_t yaw_raw = static_cast<uint16_t>(data[4]) |
                                 (static_cast<uint16_t>(data[5]) << 8U);
        const uint16_t roll_raw = static_cast<uint16_t>(data[6]) |
                                  (static_cast<uint16_t>(data[7]) << 8U);
        dm_imu_pitch_deg = DecodeUint16(pitch_raw, kPitchMinDeg, kPitchSpanDeg);
        dm_imu_yaw_deg = DecodeUint16(yaw_raw, kYawRollMinDeg, kYawRollSpanDeg);
        dm_imu_roll_deg = DecodeUint16(roll_raw, kYawRollMinDeg, kYawRollSpanDeg);
        dm_imu_last_rx_ms = HAL_GetTick();
        dm_imu_pitch_sequence = dm_imu_pitch_sequence + 1U;
        dm_imu_pitch_valid = true;
    }
    else if (data[0] == kGyroRegister)
    {
        /* 表 B-1：DATA[2..7]=X/Y/Z 角速度，小端 uint16，DATA[1] 保留。 */
        for (uint32_t axis = 0U; axis < 3U; ++axis)
        {
            const uint16_t raw = static_cast<uint16_t>(data[2U + axis * 2U]) |
                                 (static_cast<uint16_t>(data[3U + axis * 2U]) << 8U);
            dm_imu_gyro_rad_s[axis] =
                DecodeUint16(raw, kGyroMinRadS, kGyroSpanRadS);
        }
        dm_imu_gyro_sequence = dm_imu_gyro_sequence + 1U;
        dm_imu_gyro_valid = true;
    }
}
/** 按原工程格式请求一帧指定寄存器数据：0xCC + 寄存器号 + 0x00 + 0xDD。 */
static bool RequestFrame(uint8_t reg)
{
    if (dm_imu_can == nullptr)
    {
        return false;
    }

    Struct_CAN_Tx_Msg message = {};
    message.hfdcan = dm_imu_can;
    message.id = dm_imu_can_id;
    message.len = 8U;
    message.data[0] = 0xCCU;
    message.data[1] = reg;
    message.data[2] = 0x00U;
    message.data[3] = 0xDDU;

    return CAN_Tx_Submit(&message);
}

} // namespace

extern "C" bool DM_IMU_Init(FDCAN_HandleTypeDef *hfdcan,
                           uint32_t can_id,
                           uint32_t mst_id)
{
    if (hfdcan == nullptr || can_id > 0x7FFU || mst_id > 0x7FFU)
    {
        return false;
    }

    dm_imu_can = hfdcan;
    dm_imu_can_id = can_id;
    dm_imu_pitch_deg = 0.0f;
    dm_imu_pitch_valid = false;
    dm_imu_pitch_sequence = 0U;
    dm_imu_last_rx_ms = 0U;
    dm_imu_yaw_deg = 0.0f;
    dm_imu_roll_deg = 0.0f;
    dm_imu_gyro_rad_s[0] = dm_imu_gyro_rad_s[1] = dm_imu_gyro_rad_s[2] = 0.0f;
    dm_imu_gyro_valid = false;
    dm_imu_gyro_sequence = 0U;

    return BSP_CAN_RegisterCallback(mst_id,
                                    hfdcan,
                                    DM_IMU_RxCallback,
                                    nullptr);
}

extern "C" bool DM_IMU_RequestEuler(void)
{
    return RequestFrame(kEulerRegister);
}

extern "C" bool DM_IMU_RequestGyro(void)
{
    return RequestFrame(kGyroRegister);
}

extern "C" bool DM_IMU_GetPitch(float *pitch_deg)
{
    if (pitch_deg == nullptr)
    {
        return false;
    }

    *pitch_deg = dm_imu_pitch_deg;
    return dm_imu_pitch_valid &&
           (HAL_GetTick() - dm_imu_last_rx_ms) <= kPitchTimeoutMs;
}

extern "C" Struct_DM_IMU_Euler_Snapshot DM_IMU_GetEulerSnapshot(void)
{
    const uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    const Struct_DM_IMU_Euler_Snapshot snapshot{dm_imu_pitch_deg, dm_imu_yaw_deg,
                                                dm_imu_roll_deg, dm_imu_pitch_sequence, dm_imu_last_rx_ms, dm_imu_pitch_valid};
    __DMB();
    __set_PRIMASK(interrupt_state);
    return snapshot;
}

extern "C" bool DM_IMU_GetPitchSample(float *pitch_deg, uint32_t *sequence)
{
    if (pitch_deg == nullptr || sequence == nullptr)
    {
        return false;
    }

    /* 中断可能随时更新角度与序号，重试直到取到同一帧的一致快照。 */
    uint32_t sequence_before;
    uint32_t sequence_after;
    do
    {
        sequence_before = dm_imu_pitch_sequence;
        *pitch_deg = dm_imu_pitch_deg;
        sequence_after = dm_imu_pitch_sequence;
    } while (sequence_before != sequence_after);

    *sequence = sequence_after;
    return dm_imu_pitch_valid &&
           (HAL_GetTick() - dm_imu_last_rx_ms) <= kPitchTimeoutMs;
}

extern "C" bool DM_IMU_GetEuler(float *pitch_deg, float *yaw_deg, float *roll_deg)
{
    if (pitch_deg == nullptr || yaw_deg == nullptr || roll_deg == nullptr)
    {
        return false;
    }

    *pitch_deg = dm_imu_pitch_deg;
    *yaw_deg = dm_imu_yaw_deg;
    *roll_deg = dm_imu_roll_deg;
    return dm_imu_pitch_valid &&
           (HAL_GetTick() - dm_imu_last_rx_ms) <= kPitchTimeoutMs;
}

extern "C" bool DM_IMU_GetGyro(float *x_rad_s, float *y_rad_s, float *z_rad_s)
{
    if (x_rad_s == nullptr || y_rad_s == nullptr || z_rad_s == nullptr)
    {
        return false;
    }

    *x_rad_s = dm_imu_gyro_rad_s[0];
    *y_rad_s = dm_imu_gyro_rad_s[1];
    *z_rad_s = dm_imu_gyro_rad_s[2];
    return dm_imu_gyro_valid;
}

extern "C" uint32_t DM_IMU_GetLastRxMs(void)
{
    return dm_imu_last_rx_ms;
}
