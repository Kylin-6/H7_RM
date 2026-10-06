#include "transport_protocol.h"

#include <cmath>

namespace
{
bool EncodeScalar(float value, uint8_t *bytes)
{
    const float scaled = value * 1000.0f;
    if (!std::isfinite(scaled) || scaled < -32768.0f || scaled > 32767.0f)
    {
        return false;
    }
    const int16_t fixed = static_cast<int16_t>(std::lround(scaled));
    const uint16_t bits = static_cast<uint16_t>(fixed);
    bytes[0] = static_cast<uint8_t>(bits);
    bytes[1] = static_cast<uint8_t>(bits >> 8U);
    return true;
}

float DecodeScalar(const uint8_t *bytes)
{
    const uint16_t bits = static_cast<uint16_t>(bytes[0]) |
                          (static_cast<uint16_t>(bytes[1]) << 8U);
    const int32_t signed_value = bits < 0x8000U ? bits :
                                 static_cast<int32_t>(bits) - 0x10000;
    return static_cast<float>(signed_value) * 0.001f;
}
}

namespace TransportProtocol
{
bool EncodeChassisCmd(const ChassisCmd &command, uint8_t sequence,
                      uint8_t (&bytes)[kPayloadSize])
{
    const uint8_t mode = static_cast<uint8_t>(command.mode);
    if (mode > static_cast<uint8_t>(ChassisMode::ROTATE))
    {
        return false;
    }
    uint8_t encoded[kPayloadSize] = {sequence,
        static_cast<uint8_t>((kVersion << 4U) | mode)};
    if (!EncodeScalar(command.velocity_x_m_s, &encoded[2]) ||
        !EncodeScalar(command.velocity_y_m_s, &encoded[4]) ||
        !EncodeScalar(command.angular_velocity_rad_s, &encoded[6]))
    {
        return false;
    }
    for (uint8_t index = 0U; index < kPayloadSize; ++index)
    {
        bytes[index] = encoded[index];
    }
    return true;
}

bool DecodeChassisCmd(const uint8_t *bytes, uint32_t size,
                      ChassisCmd &command, uint8_t &sequence)
{
    if (bytes == nullptr || size != kPayloadSize ||
        (bytes[1] >> 4U) != kVersion ||
        (bytes[1] & 0x0FU) > static_cast<uint8_t>(ChassisMode::ROTATE))
    {
        return false;
    }
    ChassisCmd decoded{};
    decoded.mode = static_cast<ChassisMode>(bytes[1] & 0x0FU);
    decoded.velocity_x_m_s = DecodeScalar(&bytes[2]);
    decoded.velocity_y_m_s = DecodeScalar(&bytes[4]);
    decoded.angular_velocity_rad_s = DecodeScalar(&bytes[6]);
    command = decoded;
    sequence = bytes[0];
    return true;
}

bool EncodeChassisFeedback(const ChassisFeedback &feedback, uint8_t sequence,
                           uint8_t (&bytes)[kPayloadSize])
{
    uint8_t encoded[kPayloadSize] = {sequence,
        static_cast<uint8_t>((kVersion << 4U) |
                             (feedback.enabled ? 1U : 0U) |
                             (feedback.online ? 2U : 0U))};
    if (!EncodeScalar(feedback.velocity_x_m_s, &encoded[2]) ||
        !EncodeScalar(feedback.velocity_y_m_s, &encoded[4]) ||
        !EncodeScalar(feedback.angular_velocity_rad_s, &encoded[6]))
    {
        return false;
    }
    for (uint8_t index = 0U; index < kPayloadSize; ++index)
    {
        bytes[index] = encoded[index];
    }
    return true;
}

bool DecodeChassisFeedback(const uint8_t *bytes, uint32_t size,
                           ChassisFeedback &feedback, uint8_t &sequence)
{
    if (bytes == nullptr || size != kPayloadSize ||
        (bytes[1] >> 4U) != kVersion || (bytes[1] & 0x0CU) != 0U)
    {
        return false;
    }
    ChassisFeedback decoded{};
    decoded.enabled = (bytes[1] & 1U) != 0U;
    decoded.online = (bytes[1] & 2U) != 0U;
    decoded.velocity_x_m_s = DecodeScalar(&bytes[2]);
    decoded.velocity_y_m_s = DecodeScalar(&bytes[4]);
    decoded.angular_velocity_rad_s = DecodeScalar(&bytes[6]);
    feedback = decoded;
    sequence = bytes[0];
    return true;
}

bool EncodeGimbalImu(const INS_State &state, uint8_t sequence,
                     uint8_t (&attitude)[kPayloadSize], uint8_t (&rate)[kPayloadSize])
{
    constexpr float kTwoPi = 6.28318530718f;
    uint8_t angles[kPayloadSize] = {sequence, static_cast<uint8_t>(kVersion << 4U)};
    uint8_t rates[kPayloadSize] = {sequence, static_cast<uint8_t>(kVersion << 4U)};
    if (!EncodeScalar(std::remainder(state.yaw_rad, kTwoPi), &angles[2]) ||
        !EncodeScalar(std::remainder(state.pitch_rad, kTwoPi), &angles[4]) ||
        !EncodeScalar(std::remainder(state.roll_rad, kTwoPi), &angles[6]) ||
        !EncodeScalar(state.gyro_x_rad_s, &rates[2]) ||
        !EncodeScalar(state.gyro_y_rad_s, &rates[4]) ||
        !EncodeScalar(state.gyro_z_rad_s, &rates[6]))
    {
        return false;
    }
    for (uint8_t index = 0U; index < kPayloadSize; ++index)
    {
        attitude[index] = angles[index];
        rate[index] = rates[index];
    }
    return true;
}

bool DecodeGimbalImu(const uint8_t *attitude, const uint8_t *rate,
                     INS_State &state, uint8_t &sequence)
{
    if (attitude == nullptr || rate == nullptr || attitude[0] != rate[0] ||
        attitude[1] != (kVersion << 4U) || rate[1] != (kVersion << 4U))
    {
        return false;
    }
    INS_State decoded{};
    decoded.yaw_rad = DecodeScalar(&attitude[2]);
    decoded.pitch_rad = DecodeScalar(&attitude[4]);
    decoded.roll_rad = DecodeScalar(&attitude[6]);
    if (std::fabs(decoded.yaw_rad) > 3.1425f ||
        std::fabs(decoded.pitch_rad) > 3.1425f ||
        std::fabs(decoded.roll_rad) > 3.1425f)
    {
        return false;
    }
    decoded.gyro_x_rad_s = DecodeScalar(&rate[2]);
    decoded.gyro_y_rad_s = DecodeScalar(&rate[4]);
    decoded.gyro_z_rad_s = DecodeScalar(&rate[6]);
    state = decoded;
    sequence = attitude[0];
    return true;
}

bool SequenceNewer(uint8_t candidate, uint8_t previous)
{
    const uint8_t distance = static_cast<uint8_t>(candidate - previous);
    return distance != 0U && distance < 128U;
}
}
