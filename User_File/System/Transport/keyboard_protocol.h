#ifndef KEYBOARD_PROTOCOL_H
#define KEYBOARD_PROTOCOL_H

#include <cstdint>

constexpr uint32_t KEYBOARD_CONTROL_ID = 0x066U;
constexpr uint32_t KEYBOARD_CONTROL_MAX_AGE_MS = 50U;

enum class ReceiverMode : uint8_t
{
    Stop = 0,
    Remote = 1,
    Keyboard = 2
};

struct Struct_Keyboard_Frame
{
    ReceiverMode mode = ReceiverMode::Stop;
    uint8_t sequence = 0U;
    int16_t mouse_x = 0;
    uint16_t keyboard = 0U;
    bool permitted = false;
    uint32_t received_ms = 0U;
};

void KeyboardProtocol_Encode(const Struct_Keyboard_Frame& frame, uint8_t data[8]);
bool KeyboardProtocol_Decode(const uint8_t* data, uint32_t length, Struct_Keyboard_Frame& frame);
bool RemoteChannelsProtocol_Valid(const uint8_t* data, uint32_t length);

/** ISR 单写者；合法重复帧维持流量基准，但不能更新业务快照或延长许可。 */
class Class_ControlSequence
{
  public:
    bool Accept(uint8_t sequence, uint32_t received_ms);

  private:
    bool received = false;
    uint8_t last_sequence = 0U;
    uint32_t last_traffic_ms = 0U;
};

#endif
