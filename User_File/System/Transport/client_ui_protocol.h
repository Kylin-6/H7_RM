#ifndef CLIENT_UI_PROTOCOL_H
#define CLIENT_UI_PROTOCOL_H

#include <cstdint>

constexpr uint32_t CLIENT_UI_CAN_ID = 0x068U;
constexpr uint32_t CLIENT_UI_MAX_AGE_MS = 200U;
// 仅显示底盘仲裁后的模式与旋转请求，不代表电机已经执行。
struct Struct_Client_UI_Chassis
{
    uint8_t mode = 0U; // 0 停机，1 遥控，2 键鼠
    uint8_t gear = 0U; // 0 未知，1/2/3 对应遥控三档
    uint8_t spin = 0U; // 0 停止，1 正转，2 反转，3 跟随
};

// 0 未知，1 停止，2 转动；云台向底盘回传，不复用旧方向的 0x067。
inline void ClientUIProtocol_Encode(uint8_t friction, uint8_t sequence, uint8_t data[8])
{
    data[0] = 0x10U;
    data[1] = sequence;
    data[2] = friction;
    data[3] = data[4] = data[5] = data[6] = data[7] = 0U;
}

inline bool ClientUIProtocol_Decode(const uint8_t *data, uint32_t length, uint8_t &friction)
{
    if (data == nullptr || length != 8U || data[0] != 0x10U || data[2] > 2U ||
        data[3] != 0U || data[4] != 0U || data[5] != 0U || data[6] != 0U || data[7] != 0U)
        return false;
    friction = data[2];
    return true;
}
#endif
