#include "Gimbal.h"
#include "Shoot_Config.h"
#include "input_state.h"
#include "keyboard_input.h"
#include "keyboard_link.h"
#include "keyboard_logic.h"
#include "message_center.h"
#include "remote_input.h"
#include "usart.h"
#include "vtm_legacy.h"

namespace
{
bool initialized;
ReceiverMode last_mode = ReceiverMode::Stop;
bool last_permitted;
bool was_ready;
bool captured;
float pitch_target;
uint32_t last_update_ms;
uint32_t last_send_ms;
uint32_t event_sequence;
KeyboardInput::Class_MouseTrigger trigger;
} // namespace

bool KeyboardInput_Init()
{
    if (initialized)
        return true;
    initialized = VTM_Legacy_Init(&huart7) && KeyboardLink_Init(false);
    last_update_ms = HAL_GetTick();
    return initialized;
}

void KeyboardInput_Update()
{
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed = now - last_update_ms;
    last_update_ms = now;
    InputState_SetTime(now);
    Struct_VTM_Legacy_Keyboard_Snapshot rc{};
    const bool received = initialized && VTM_Legacy_ReadKeyboardSnapshot(&rc);
    ReceiverMode requested = ReceiverMode::Stop;
    const bool receiver_permitted = RemoteInput_GetReceiverState(requested);
    const bool keyboard_fresh = received && now - rc.received_ms <= KEYBOARD_CONTROL_MAX_AGE_MS;
    const ReceiverMode mode = !initialized || (requested == ReceiverMode::Keyboard && !keyboard_fresh)
                                  ? ReceiverMode::Stop : requested;
    const auto config = Gimbal_Default_Config();
    INS_State ins{};
    const bool ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, config.ins_max_age_us) && std::isfinite(ins.pitch_rad);
    const bool permitted = receiver_permitted && (mode == ReceiverMode::Remote ||
                                                  (mode == ReceiverMode::Keyboard && ins_valid));
    const bool transition = mode != last_mode || permitted != last_permitted;
    InputState_Select(mode == ReceiverMode::Keyboard ? InputSource::Keyboard : InputSource::Remote, false, now);
    InputState_SetPermission(permitted, mode != ReceiverMode::Keyboard);

    const bool ready = permitted && mode == ReceiverMode::Keyboard && Gimbal_GetStatus() == Gimbal_Status_READY;
    if (transition || !ready)
    {
        trigger.Reset();
        captured = false;
    }
    ControlInput input{};
    input.shoot_event_sequence = event_sequence;
    if (mode == ReceiverMode::Keyboard && permitted)
    {
        // 提交 IMU 保持目标才能启动现有 2 s 使能流程；未 ready 时始终跟住新鲜姿态。
        const bool entering = !captured || !was_ready;
        if (entering)
        {
            pitch_target = std::fmax(config.pitch_min, std::fmin(config.pitch_max, ins.pitch_rad));
            captured = true;
        }
        else
        {
            const float speed = KeyboardInput::PitchSpeed(rc.mouse_y, config.keyboard_pitch_mouse_max,
                                                          config.keyboard_pitch_mouse_divisor, config.pitch_torque.target_rate_rad_s);
            pitch_target = KeyboardInput::IntegratePitch(pitch_target, speed, elapsed,
                                                         config.keyboard_pitch_max_dt_ms, config.pitch_min, config.pitch_max);
        }
        input.gimbal.mode = GimbalMode::IMU;
        input.gimbal.pitch_angle_rad = pitch_target;
        if (ready)
        {
            if (trigger.Update(rc.mouse_left != 0U, now, InfantryShootConfig::KEYBOARD_LONG_PRESS_MS))
                ++event_sequence;
            input.shoot_event_sequence = event_sequence;
            input.shoot.shoot_mode = ShootMode::ON;
            input.shoot.friction_mode = trigger.Pressed() ? FrictionMode::ON : FrictionMode::OFF;
            input.shoot.loader_mode = trigger.Burst(now, InfantryShootConfig::KEYBOARD_LONG_PRESS_MS) ? LoaderMode::BURST : LoaderMode::STOP;
            input.shoot.loader_speed_rad_s = InfantryShootConfig::LOADER_BURST_OUTPUT_RAD_S;
        }
        input.received_ms = rc.received_ms;
        input.valid = true;
    }
    InputState_SubmitKeyboard(input);
    // 模式/许可变化立即提交；失败时下个控制周期重试，不当作对端确认。
    if (transition || now - last_send_ms >= 10U)
    {
        Struct_Keyboard_Frame frame{};
        frame.mode = mode;
        frame.mouse_x = rc.mouse_x;
        frame.keyboard = rc.keyboard;
        frame.permitted = permitted;
        if (KeyboardLink_Send(frame))
        {
            last_send_ms = now;
            last_mode = mode;
            last_permitted = permitted;
        }
    }
    was_ready = ready;
}
