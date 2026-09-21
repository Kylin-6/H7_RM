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

#include <stddef.h>

namespace
{
/** 欧拉角寄存器号；数据帧首字节必须等于该值。 */
constexpr uint8_t kEulerRegister = 0x03U;
/** Pitch 解码下限，单位度。 */
constexpr float kPitchMinDeg = -90.0f;
/** Pitch 解码跨度，单位度。 */
constexpr float kPitchSpanDeg = 180.0f;
/** uint16 满量程，用于归一化解码。 */
constexpr float kUint16Max = 65535.0f;

FDCAN_HandleTypeDef *dm_imu_can;
uint32_t dm_imu_can_id;
volatile float dm_imu_pitch_deg;
volatile bool dm_imu_pitch_valid;
volatile uint32_t dm_imu_pitch_sequence;

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

    if (data == nullptr || len < 8U || data[0] != kEulerRegister)
    {
        return;
    }

    const uint16_t pitch_raw = static_cast<uint16_t>(data[2]) |
                               (static_cast<uint16_t>(data[3]) << 8U);
    dm_imu_pitch_deg = DecodeUint16(pitch_raw, kPitchMinDeg, kPitchSpanDeg);
    dm_imu_pitch_sequence = dm_imu_pitch_sequence + 1U;
    dm_imu_pitch_valid = true;
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

    return BSP_CAN_RegisterCallback(mst_id,
                                    hfdcan,
                                    DM_IMU_RxCallback,
                                    nullptr);
}

extern "C" bool DM_IMU_RequestEuler(void)
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
    message.data[1] = kEulerRegister;
    message.data[2] = 0x00U;
    message.data[3] = 0xDDU;

    return CAN_Tx_Submit(&message);
}

extern "C" bool DM_IMU_GetPitch(float *pitch_deg)
{
    if (pitch_deg == nullptr)
    {
        return false;
    }

    *pitch_deg = dm_imu_pitch_deg;
    return dm_imu_pitch_valid;
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
    return dm_imu_pitch_valid;
}
