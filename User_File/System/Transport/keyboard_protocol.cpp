#include "keyboard_protocol.h"

void KeyboardProtocol_Encode(const Struct_Keyboard_Frame& frame, uint8_t data[8])
{
    const bool permitted = frame.permitted && frame.mode != ReceiverMode::Stop;
    const int16_t x = !permitted ? 0 : frame.mouse_x > 1000 ? 1000
                                   : frame.mouse_x < -1000  ? -1000
                                                            : frame.mouse_x;
    const uint16_t keys = permitted ? frame.keyboard : 0U;
    data[0] = 0x10U | static_cast<uint8_t>(frame.mode);
    data[1] = frame.sequence;
    data[2] = static_cast<uint16_t>(x) >> 8U;
    data[3] = static_cast<uint8_t>(x);
    data[4] = keys >> 8U;
    data[5] = static_cast<uint8_t>(keys);
    data[6] = permitted ? 1U : 0U;
    data[7] = 0U;
}

bool KeyboardProtocol_Decode(const uint8_t* data, uint32_t length, Struct_Keyboard_Frame& frame)
{
    if (data == nullptr || length != 8U || (data[0] & 0xF0U) != 0x10U ||
        (data[0] & 0x0FU) > 2U || (data[6] & 0xFEU) != 0U || data[7] != 0U)
        return false;
    Struct_Keyboard_Frame decoded{};
    decoded.mode = static_cast<ReceiverMode>(data[0] & 0x0FU);
    decoded.sequence = data[1];
    decoded.mouse_x = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8U) | data[3]);
    decoded.keyboard = (static_cast<uint16_t>(data[4]) << 8U) | data[5];
    decoded.permitted = data[6] != 0U;
    if (decoded.mouse_x < -1000 || decoded.mouse_x > 1000 ||
        (decoded.mode == ReceiverMode::Stop && decoded.permitted) ||
        (!decoded.permitted && (decoded.mouse_x != 0 || decoded.keyboard != 0U)))
        return false;
    frame = decoded;
    return true;
}

bool RemoteChannelsProtocol_Valid(const uint8_t* data, uint32_t length)
{
    if (data == nullptr || length != 8U || (data[6] & 0xF8U) != 0x10U)
        return false;
    const auto mode = static_cast<ReceiverMode>((data[6] >> 1U) & 3U);
    if (mode > ReceiverMode::Keyboard || (mode == ReceiverMode::Stop && (data[6] & 1U)))
        return false;
    // 只有已许可的遥控档携带摇杆，键鼠档和停机帧不能留下旧火控通道。
    if (mode != ReceiverMode::Remote || (data[6] & 1U) == 0U)
        for (unsigned i = 0; i < 6U; ++i)
            if (data[i] != 0U)
                return false;
    return true;
}

bool Class_ControlSequence::Accept(uint8_t sequence, uint32_t received_ms)
{
    const bool reset = !received || received_ms - last_traffic_ms > KEYBOARD_CONTROL_MAX_AGE_MS;
    last_traffic_ms = received_ms;
    const uint8_t distance = static_cast<uint8_t>(sequence - last_sequence);
    if (!reset && (distance == 0U || distance >= 128U))
        return false;
    received = true;
    last_sequence = sequence;
    return true;
}
