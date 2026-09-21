/**
 * @file chassis_board.cpp
 * @brief 底盘板到云台板板间链路的解码与超时判定。
 * @author Kylin-6（原始实现）/ H7_BSP 移植
 */

#include "chassis_board.h"

#include "stm32h7xx_hal.h"

#include <stddef.h>

namespace
{
/** 大端 int16 解码；帧内通道字节序与底盘板原有的组织方式一致。 */
int16_t DecodeI16BigEndian(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                static_cast<uint16_t>(data[1]));
}
} // namespace

bool Class_ChassisBoard::Init(FDCAN_HandleTypeDef *motor_hfdcan)
{
    if (initialized || motor_hfdcan == nullptr)
    {
        return initialized;
    }
    if (!BSP_CAN_RegisterCallback(CHASSIS_BOARD_ID_REMOTE_CHANNELS,
                                  motor_hfdcan,
                                  RxCallback,
                                  this))
    {
        return false;
    }

    hfdcan = motor_hfdcan;
    fire_channel = 0;
    dial_channel = 0;
    pitch_channel = 0;
    last_rx_ms = 0U;
    received = false;
    initialized = true;
    return true;
}

bool Class_ChassisBoard::GetFire(int16_t *value)
{
    return ReadChannel(fire_channel, value);
}

bool Class_ChassisBoard::GetDial(int16_t *value)
{
    return ReadChannel(dial_channel, value);
}

bool Class_ChassisBoard::GetPitch(int16_t *value)
{
    return ReadChannel(pitch_channel, value);
}

bool Class_ChassisBoard::IsOnline() const
{
    return received &&
           (HAL_GetTick() - last_rx_ms) <= CHASSIS_BOARD_CHANNEL_TIMEOUT_MS;
}

void Class_ChassisBoard::RxCallback(FDCAN_HandleTypeDef *callback_hfdcan,
                                    uint32_t id,
                                    uint8_t *data,
                                    uint32_t len,
                                    void *context)
{
    Class_ChassisBoard *board = static_cast<Class_ChassisBoard *>(context);
    if (board == nullptr || callback_hfdcan != board->hfdcan || data == nullptr)
    {
        return;
    }

    if (id == CHASSIS_BOARD_ID_REMOTE_CHANNELS)
    {
        board->OnRemoteChannels(data, len);
    }
}

void Class_ChassisBoard::OnRemoteChannels(const uint8_t *data, uint32_t len)
{
    /* 帧内至少包含火控、波轮、Pitch 三个通道。 */
    if (len < 6U)
    {
        return;
    }

    fire_channel = DecodeI16BigEndian(&data[0]);
    dial_channel = DecodeI16BigEndian(&data[2]);
    pitch_channel = DecodeI16BigEndian(&data[4]);
    last_rx_ms = HAL_GetTick();
    received = true;
}

bool Class_ChassisBoard::ReadChannel(const volatile int16_t &source,
                                     int16_t *value) const
{
    if (value == nullptr || !IsOnline())
    {
        return false;
    }
    *value = source;
    return true;
}
