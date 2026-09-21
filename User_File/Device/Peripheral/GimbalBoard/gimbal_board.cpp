/**
 * @file gimbal_board.cpp
 * @brief 底盘板到云台板的板间 CAN 链路实现（老步兵配置）。
 */

#include "gimbal_board.h"

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

bool Class_GimbalBoard::SendRemoteChannels(
    const int16_t sbus_channels[GIMBAL_BOARD_CHANNEL_COUNT])
{
    uint8_t data[8] = {0};

    /* 火控（发射）开关 -> data[0..1]；极性由 GIMBAL_BOARD_FIRE_SWITCH_INVERT 决定。 */
    int16_t fire_switch = sbus_channels[GIMBAL_BOARD_CHANNEL_FIRE_SWITCH];
#if GIMBAL_BOARD_FIRE_SWITCH_INVERT
    fire_switch = (int16_t)(-fire_switch);
#endif
    data[0] = (uint8_t)((uint16_t)fire_switch >> 8);
    data[1] = (uint8_t)fire_switch;
    /* 发射速度 -> data[2..3] */
    const int16_t shoot_speed = sbus_channels[GIMBAL_BOARD_CHANNEL_SHOOT_SPEED];
    data[2] = (uint8_t)((uint16_t)shoot_speed >> 8);
    data[3] = (uint8_t)shoot_speed;
    /* Pitch 轴 -> data[4..5] */
    const int16_t pitch = sbus_channels[GIMBAL_BOARD_CHANNEL_PITCH];
    data[4] = (uint8_t)((uint16_t)pitch >> 8);
    data[5] = (uint8_t)pitch;

    return Transmit(GIMBAL_BOARD_ID_REMOTE_CHANNELS, data);
}

bool Class_GimbalBoard::SendChassisYaw(float dm_yaw_rad, float ground_yaw_rad)
{
    uint8_t data[8] = {0};
    const int16_t yaw_int =
        (int16_t)((dm_yaw_rad - GIMBAL_BOARD_YAW_OFFSET_DEG) * GIMBAL_BOARD_YAW_SCALE);
    const int16_t ground_yaw_int =
        (int16_t)((ground_yaw_rad - GIMBAL_BOARD_YAW_OFFSET_DEG) * GIMBAL_BOARD_YAW_SCALE);

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
