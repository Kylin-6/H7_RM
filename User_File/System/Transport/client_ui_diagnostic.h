#ifndef CLIENT_UI_DIAGNOSTIC_H
#define CLIENT_UI_DIAGNOSTIC_H

#include <cstdint>

constexpr uint32_t CLIENT_UI_DIAGNOSTIC_ID = 0x069U;
constexpr uint8_t CLIENT_UI_INPUT_RECEIVER = 1U << 1U;
constexpr uint8_t CLIENT_UI_INPUT_FRAME = 1U << 2U;
constexpr uint8_t CLIENT_UI_INPUT_INS = 1U << 3U;
constexpr uint8_t CLIENT_UI_INPUT_OVERFLOW = 1U << 4U;

// 故障位沿用各板诊断 Topic；仅 Gimbal 的快照在线上传输，不参与控制许可。
struct Struct_Client_UI_Diagnostic
{
    uint32_t fault_mask = 0U;
    uint8_t input_reason = 0U;
    bool permitted = false;
    bool waiting = false;
    bool control_fresh = false;
    bool snapshot_valid = false;
};

inline void ClientUIDiagnostic_Encode(const Struct_Client_UI_Diagnostic& state, uint8_t sequence, uint8_t data[8])
{
    data[0] = 0x10U;
    data[1] = sequence;
    for (uint8_t i = 0U; i < 4U; ++i)
        data[2U + i] = static_cast<uint8_t>(state.fault_mask >> (8U * i));
    data[6] = (state.permitted ? 1U : 0U) | (state.waiting ? 2U : 0U) |
              (state.control_fresh ? 4U : 0U) | (state.snapshot_valid ? 8U : 0U);
    data[7] = state.input_reason;
}

inline bool ClientUIDiagnostic_Decode(const uint8_t* data, uint32_t length, Struct_Client_UI_Diagnostic& state)
{
    if (data == nullptr || length != 8U || data[0] != 0x10U || (data[6] & 0xF0U) != 0U ||
        (data[7] & ~0x1EU) != 0U || ((data[6] & 4U) != 0U && (data[6] & 8U) == 0U))
        return false;
    Struct_Client_UI_Diagnostic next{};
    for (uint8_t i = 0U; i < 4U; ++i)
        next.fault_mask |= uint32_t(data[2U + i]) << (8U * i);
    next.permitted = (data[6] & 1U) != 0U;
    next.waiting = (data[6] & 2U) != 0U;
    next.control_fresh = (data[6] & 4U) != 0U;
    next.snapshot_valid = (data[6] & 8U) != 0U;
    next.input_reason = data[7];
    state = next;
    return true;
}
#endif
