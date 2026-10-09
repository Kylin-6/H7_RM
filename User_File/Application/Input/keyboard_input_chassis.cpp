#include "RobotCmd.h"
#include "remote_input.h"
#include "keyboard_chassis_mapping.h"
#include "keyboard_input.h"
#include "keyboard_link.h"
#include "stm32h7xx_hal.h"

namespace
{
bool initialized;
}

bool KeyboardInput_Init()
{
    initialized = KeyboardLink_Init(true);
    return initialized;
}

void KeyboardInput_Update()
{
    const uint32_t now = HAL_GetTick();
    InputState_SetTime(now);
    Struct_Keyboard_Frame frame{};
    const bool received = initialized && KeyboardLink_Read(frame);
    ReceiverMode requested = ReceiverMode::Stop;
    const bool receiver_permitted = RemoteInput_GetReceiverState(requested);
    const ReceiverMode forwarded = initialized ? KeyboardInput::SelectMode(received, frame.received_ms, now, static_cast<uint8_t>(frame.mode)) : ReceiverMode::Stop;
    // 未建立云台链路时仅允许原遥控；键鼠必须同时满足本板 CH5 与云台反馈。
    const ReceiverMode mode = forwarded == requested ? forwarded : ReceiverMode::Stop;
    const bool permitted = receiver_permitted && mode != ReceiverMode::Stop &&
                           (!received || frame.permitted);
    InputState_Select(mode == ReceiverMode::Keyboard ? InputSource::Keyboard : InputSource::Remote, false, now);
    InputState_SetPermission(permitted, mode != ReceiverMode::Keyboard);
    ControlInput input{};
    if (mode == ReceiverMode::Keyboard && permitted)
    {
        GimbalFeedback feedback{};
        const bool yaw_valid = RobotCmd_GetGimbalFeedback(feedback) && feedback.enabled && std::isfinite(feedback.yaw_rad);
        input = KeyboardChassis_Map(frame.keyboard, frame.mouse_x, yaw_valid, feedback.yaw_rad);
        input.received_ms = frame.received_ms;
        input.valid = true;
    }
    InputState_SubmitKeyboard(input);
}
