#ifndef KEYBOARD_INPUT_H
#define KEYBOARD_INPUT_H

#include <stdint.h>

enum Enum_Keyboard_Stop_Reason : uint32_t
{
    KEYBOARD_STOP_SOURCE_CHANGE = 1U << 0,
    KEYBOARD_STOP_RECEIVER = 1U << 1,
    KEYBOARD_STOP_FRAME = 1U << 2,
    KEYBOARD_STOP_INS = 1U << 3
};

struct Struct_Keyboard_Stop_Record
{
    uint32_t timestamp_ms, reason, keyboard_age_ms, keyboard_sequence;
};

struct Struct_Keyboard_Input_Debug
{
    uint32_t tick_ms, keyboard_age_ms, keyboard_sequence, stop_count;
    uint8_t requested_mode, mode, gimbal_status;
    bool receiver_permitted, keyboard_fresh, ins_valid, permitted;
    Struct_Keyboard_Stop_Record stops[8];
};

/** 老步兵云台 ControlTask 唯一写入；调试记录，不参与控制。 */
extern Struct_Keyboard_Input_Debug Keyboard_Input_Debug;

bool KeyboardInput_Init();
void KeyboardInput_Update();

#endif
