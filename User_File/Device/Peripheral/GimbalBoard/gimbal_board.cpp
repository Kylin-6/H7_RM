/**
 * @file gimbal_board.cpp
 * @brief 底盘板到云台板的板间 CAN 链路实现（老步兵配置）。
 */

#include "gimbal_board.h"

#include "alg_basic.h"

bool Class_GimbalBoard::Init(FDCAN_HandleTypeDef* motor_hfdcan)
{
    if (motor_hfdcan == nullptr)
    {
        return false;
    }

    hfdcan = motor_hfdcan;
    return true;
}

bool Class_GimbalBoard::Transmit(uint32_t id, const uint8_t data[8])
{
    if (hfdcan == nullptr)
    {
        return false;
    }

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = id;
    message.len = 8U;

    for (uint32_t index = 0U; index < 8U; ++index)
    {
        message.data[index] = data[index];
    }

    /* 板间状态帧是周期量，用 Latest-Value 通道，避免堆叠历史帧。 */
    return CAN_Tx_Perform(&message);
}

bool Class_GimbalBoard::SendRemoteChannels(int16_t fire_switch,
                                           int16_t shoot_speed,
                                           int16_t pitch, ReceiverMode mode, bool permitted)
{
    uint8_t data[8] = {0};

    /* 火控（发射）开关 -> data[0..1] */
    data[0] = (uint8_t)((uint16_t)fire_switch >> 8);
    data[1] = (uint8_t)fire_switch;
    /* 发射速度 -> data[2..3] */
    data[2] = (uint8_t)((uint16_t)shoot_speed >> 8);
    data[3] = (uint8_t)shoot_speed;
    /* Pitch 轴 -> data[4..5] */
    data[4] = (uint8_t)((uint16_t)pitch >> 8);
    data[5] = (uint8_t)pitch;

    data[6] = 0x10U | (static_cast<uint8_t>(mode) << 1U) | (permitted ? 1U : 0U);
    data[7] = ++remote_sequence;
    return Transmit(GIMBAL_BOARD_ID_REMOTE_CHANNELS, data);
}

namespace
{
/** 0x070 线上编码：int16 = (yaw_deg - 180) * 100，单位 0.01°。
 *  rad → degree 只在本协议 Encode 边界发生一次；编码前夹到 int16 值域，
 *  避免超出预期机械范围时静默回绕成错误的另一侧角度。 */
int16_t EncodeYawHundredthDegree(float yaw_rad)
{
    constexpr float RAD_TO_DEG = 57.29577951F;
    constexpr float INT16_MIN_VALUE = -32768.0F;
    constexpr float INT16_MAX_VALUE = 32767.0F;

    const float scaled = (yaw_rad * RAD_TO_DEG - GIMBAL_BOARD_YAW_OFFSET_DEG) *
                         GIMBAL_BOARD_YAW_SCALE;
    return (int16_t)Basic_Math_Constrain(scaled, INT16_MIN_VALUE, INT16_MAX_VALUE);
}
} // namespace

bool Class_GimbalBoard::SendChassisYaw(float dm_yaw_rad, float ground_yaw_rad)
{
    uint8_t data[8] = {0};

    /* DM 云台机械安装角为 180°，正常工作时两个字段都落在 int16 值域内。 */
    const int16_t yaw_int = EncodeYawHundredthDegree(dm_yaw_rad);
    const int16_t ground_yaw_int = EncodeYawHundredthDegree(ground_yaw_rad);

    data[0] = (uint8_t)((uint16_t)yaw_int >> 8);
    data[1] = (uint8_t)((uint16_t)yaw_int);
    data[2] = (uint8_t)((uint16_t)ground_yaw_int >> 8);
    data[3] = (uint8_t)((uint16_t)ground_yaw_int);

    return Transmit(GIMBAL_BOARD_ID_CHASSIS_YAW, data);
}

bool Class_GimbalBoard::SendRobotStatus(uint16_t heat_limit,
                                        uint16_t cooling,
                                        uint8_t robot_id)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(heat_limit >> 8);
    data[1] = (uint8_t)heat_limit;
    data[2] = (uint8_t)(cooling >> 8);
    data[3] = (uint8_t)cooling;
    data[4] = robot_id;

    return Transmit(GIMBAL_BOARD_ID_ROBOT_STATUS, data);
}
