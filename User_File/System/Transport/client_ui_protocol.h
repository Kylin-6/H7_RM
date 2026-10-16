#ifndef CLIENT_UI_PROTOCOL_H
#define CLIENT_UI_PROTOCOL_H

#include <cstdint>
#include <cmath>

constexpr uint32_t CLIENT_UI_CAN_ID = 0x068U;
constexpr uint32_t CLIENT_UI_MAX_AGE_MS = 200U;
// 旧文字 UI 的本地快照；不在线上传输，不代表电机已经执行。
struct Struct_Client_UI_Chassis
{
    uint8_t mode = 0U; // 0 停机，1 遥控，2 键鼠
    uint8_t gear = 0U; // 0 未知，1/2/3 对应遥控三档
    uint8_t spin = 0U; // 0 停止，1 正转，2 反转，3 跟随
};

// 仅供显示；速度为反馈绝对值 rad/s，不代表摩擦轮已达发射转速。
struct Struct_Client_UI_Gimbal
{
    uint8_t friction = 0U; // 0 未知，1 停止，2 转动
    uint8_t loader = 0U; // 0 未知，1 停止，2 转动
    uint8_t autoaim = 0U; // 0 未知，1 关闭，2 等待目标/许可，3 仲裁采用视觉目标
    bool speed_valid = false;
    float left_speed_rad_s = 0.0f;
    float right_speed_rad_s = 0.0f;
};

// 版本 2：状态打包，左右转速为小端 uint16，单位 0.1 rad/s。
inline bool ClientUIProtocol_Encode(const Struct_Client_UI_Gimbal &state, uint8_t sequence, uint8_t data[8])
{
    if (state.friction > 2U || state.loader > 2U || state.autoaim > 3U ||
        (state.speed_valid && (state.friction == 0U ||
         !std::isfinite(state.left_speed_rad_s) || !std::isfinite(state.right_speed_rad_s) ||
         state.left_speed_rad_s < 0.0f || state.left_speed_rad_s > 6553.5f ||
         state.right_speed_rad_s < 0.0f || state.right_speed_rad_s > 6553.5f)))
        return false;
    data[0] = 0x20U;
    data[1] = sequence;
    data[2] = state.friction | (state.loader << 2U) | (state.autoaim << 4U) | (state.speed_valid ? 0x40U : 0U);
    data[3] = 0U;
    const uint16_t left = state.speed_valid ? static_cast<uint16_t>(std::lround(state.left_speed_rad_s * 10.0f)) : 0U;
    const uint16_t right = state.speed_valid ? static_cast<uint16_t>(std::lround(state.right_speed_rad_s * 10.0f)) : 0U;
    data[4] = static_cast<uint8_t>(left);
    data[5] = static_cast<uint8_t>(left >> 8U);
    data[6] = static_cast<uint8_t>(right);
    data[7] = static_cast<uint8_t>(right >> 8U);
    return true;
}

inline bool ClientUIProtocol_Decode(const uint8_t *data, uint32_t length, Struct_Client_UI_Gimbal &state)
{
    if (data == nullptr || length != 8U)
        return false;
    Struct_Client_UI_Gimbal next{};
    if (data[0] == 0x10U) // 兼容旧云台：仅摩擦轮状态，其余保持未知。
    {
        if (data[2] > 2U || data[3] != 0U || data[4] != 0U || data[5] != 0U || data[6] != 0U || data[7] != 0U)
            return false;
        next.friction = data[2];
    }
    else if (data[0] == 0x20U)
    {
        next.friction = data[2] & 3U;
        next.loader = (data[2] >> 2U) & 3U;
        next.autoaim = (data[2] >> 4U) & 3U;
        next.speed_valid = (data[2] & 0x40U) != 0U;
        if ((data[2] & 0x80U) != 0U || data[3] != 0U || next.friction > 2U || next.loader > 2U ||
            (next.speed_valid && next.friction == 0U) ||
            (!next.speed_valid && (data[4] != 0U || data[5] != 0U || data[6] != 0U || data[7] != 0U)))
            return false;
        next.left_speed_rad_s = static_cast<float>(uint16_t(data[4]) | (uint16_t(data[5]) << 8U)) * 0.1f;
        next.right_speed_rad_s = static_cast<float>(uint16_t(data[6]) | (uint16_t(data[7]) << 8U)) * 0.1f;
    }
    else
        return false;
    state = next;
    return true;
}
#endif
