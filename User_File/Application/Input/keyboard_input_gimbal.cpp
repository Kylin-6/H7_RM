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

Struct_Keyboard_Input_Debug Keyboard_Input_Debug{};

namespace
{
bool initialized;
ReceiverMode last_mode = ReceiverMode::Stop;
bool last_permitted;
bool was_ready;
bool captured;
float pitch_target;
uint16_t yaw_counts;
uint32_t seen_overflows;
uint32_t blocked_sequence;
bool overflow_blocked;
bool had_received;
uint32_t last_frame_ms;
uint32_t last_send_ms;
uint32_t event_sequence;
KeyboardInput::Class_MouseTrigger trigger;
} // namespace

bool KeyboardInput_Init()
{
    if (initialized)
        return true;
    initialized = VTM_Legacy_Init(&huart7) && KeyboardLink_Init(false);
    return initialized;
}

void KeyboardInput_Update()
{
    Struct_VTM_Legacy_Keyboard_Batch batch{};
    if (initialized)
        VTM_Legacy_TakeKeyboardBatch(&batch);
    const auto& rc = batch.latest;
    const bool received = initialized && batch.received;
    // 批次与最新快照一次复制，再取时钟，避免跨 tick 误判超时。
    const uint32_t now = HAL_GetTick();
    if (batch.overflow_count != seen_overflows)
    {
        seen_overflows = batch.overflow_count;
        blocked_sequence = rc.sequence;
        overflow_blocked = true;
    }
    else if (overflow_blocked && rc.sequence != blocked_sequence)
        overflow_blocked = false;
    const bool input_gap = received && had_received && rc.received_ms - last_frame_ms > INPUT_KEYBOARD_MAX_AGE_MS;
    had_received = received;
    last_frame_ms = rc.received_ms;
    InputState_SetTime(now);
    ReceiverMode requested = ReceiverMode::Stop;
    const bool receiver_permitted = RemoteInput_GetReceiverState(requested);
    const bool keyboard_fresh = received && !overflow_blocked && now - rc.received_ms <= INPUT_KEYBOARD_MAX_AGE_MS;
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

    const auto gimbal_status = Gimbal_GetStatus();
    const bool ready = permitted && mode == ReceiverMode::Keyboard && gimbal_status == Gimbal_Status_READY;
    // 记录撤销键鼠许可的边沿，保留最近多次原因，避免后续停机覆盖第一次故障。
    auto &debug = Keyboard_Input_Debug;
    const uint32_t keyboard_age = received ? now - rc.received_ms : UINT32_MAX;
    if (debug.mode == static_cast<uint8_t>(ReceiverMode::Keyboard) && debug.permitted &&
        !(mode == ReceiverMode::Keyboard && permitted))
    {
        uint32_t reason = 0U;
        if (requested == ReceiverMode::Remote)
            reason |= KEYBOARD_STOP_SOURCE_CHANGE;
        if (!receiver_permitted || requested == ReceiverMode::Stop)
            reason |= KEYBOARD_STOP_RECEIVER;
        if (requested == ReceiverMode::Keyboard && !keyboard_fresh)
            reason |= KEYBOARD_STOP_FRAME;
        if (overflow_blocked)
            reason |= KEYBOARD_STOP_OVERFLOW;
        if (requested == ReceiverMode::Keyboard && !ins_valid)
            reason |= KEYBOARD_STOP_INS;
        debug.stops[debug.stop_count % 8U] = {now, reason, keyboard_age, rc.sequence};
        ++debug.stop_count;
    }
    debug.reset_reason = (transition ? KEYBOARD_STOP_SOURCE_CHANGE : 0U) |
                         (!receiver_permitted ? KEYBOARD_STOP_RECEIVER : 0U) |
                         (!keyboard_fresh || input_gap ? KEYBOARD_STOP_FRAME : 0U) |
                         (!ins_valid ? KEYBOARD_STOP_INS : 0U) |
                         (!ready ? KEYBOARD_RESET_NOT_READY : 0U) |
                         (overflow_blocked ? KEYBOARD_STOP_OVERFLOW : 0U);
    debug.overflow_count = batch.overflow_count;
    if (batch.count != 0U)
        debug.consumed_sequence = batch.frames[batch.count - 1U].sequence;
    debug.pitch_delta_rad = 0.0f;
    debug.tick_ms = now;
    debug.keyboard_age_ms = keyboard_age;
    debug.keyboard_sequence = rc.sequence;
    debug.requested_mode = static_cast<uint8_t>(requested);
    debug.mode = static_cast<uint8_t>(mode);
    debug.gimbal_status = static_cast<uint8_t>(gimbal_status);
    debug.receiver_permitted = receiver_permitted;
    debug.keyboard_fresh = keyboard_fresh;
    debug.ins_valid = ins_valid;
    debug.permitted = permitted;
    if (transition || !ready || input_gap)
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
        // 进入/恢复的批次只建立基准；未 ready 不积累任何鼠标或发射动作。
        for (uint8_t i = 0U; i < batch.count; ++i)
        {
            const auto& sample = batch.frames[i];
            if (!ready || entering || now - sample.received_ms > INPUT_KEYBOARD_MAX_AGE_MS)
                continue;
            const float delta = KeyboardInput::PitchIncrement(sample.mouse_y, config.keyboard_pitch_mouse_max,
                                                              config.keyboard_pitch_mouse_divisor, config.keyboard_pitch_rad_per_count);
            pitch_target = std::fmax(config.pitch_min, std::fmin(config.pitch_max, pitch_target + delta));
            debug.pitch_delta_rad += delta;
            const int16_t x = sample.mouse_x > config.keyboard_yaw_mouse_max ? config.keyboard_yaw_mouse_max : sample.mouse_x < -config.keyboard_yaw_mouse_max ? -config.keyboard_yaw_mouse_max
                                                                                                                                                               : sample.mouse_x;
            yaw_counts = static_cast<uint16_t>(yaw_counts + x);
            if (trigger.Update(sample.mouse_left != 0U, sample.received_ms, InfantryShootConfig::KEYBOARD_LONG_PRESS_MS))
                ++event_sequence;
        }
        input.gimbal.mode = GimbalMode::IMU;
        input.gimbal.pitch_angle_rad = pitch_target;
        if (ready)
        {
            input.shoot_event_sequence = event_sequence;
            input.shoot.shoot_mode = ShootMode::ON;
            input.shoot.friction_mode = trigger.Pressed() ? FrictionMode::ON : FrictionMode::OFF;
            input.shoot.loader_mode = trigger.Burst(now, InfantryShootConfig::KEYBOARD_LONG_PRESS_MS) ? LoaderMode::BURST : LoaderMode::STOP;
            input.shoot.loader_speed_rad_s = InfantryShootConfig::LOADER_BURST_OUTPUT_RAD_S;
        }
        input.received_ms = rc.received_ms;
        input.valid = true;
    }
    // 内部累计值跨停机保留；仅协议编码清零安全帧。
    // 即使瞬态 STOP 帧被最新值槽覆盖，对端也不会把计数复位误当成反向鼠标。
    debug.yaw_counts = yaw_counts;
    debug.pitch_target_rad = pitch_target;
    InputState_SubmitKeyboard(input);
    // 模式/许可变化立即提交；失败时下个控制周期重试，不当作对端确认。
    if (transition || now - last_send_ms >= 10U)
    {
        Struct_Keyboard_Frame frame{};
        frame.mode = mode;
        frame.yaw_counts = yaw_counts;
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
