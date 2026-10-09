#ifndef KEYBOARD_LOGIC_H
#define KEYBOARD_LOGIC_H

#include "keyboard_protocol.h"

#include <cmath>

namespace KeyboardInput
{
enum Key : uint16_t
{
    W = 1U << 0,
    S = 1U << 1,
    A = 1U << 2,
    D = 1U << 3,
    Shift = 1U << 4,
    Ctrl = 1U << 5,
    Q = 1U << 6,
    E = 1U << 7
};

inline ReceiverMode SelectMode(bool received, uint32_t timestamp, uint32_t now, uint8_t mode)
{
    if (!received)
        return ReceiverMode::Remote;
    if (now - timestamp > KEYBOARD_CONTROL_MAX_AGE_MS || mode > 2U)
        return ReceiverMode::Stop;
    return static_cast<ReceiverMode>(mode);
}

inline bool Combination(uint16_t keys)
{
    return (keys & (Ctrl | Shift)) == (Ctrl | Shift);
}

inline float PitchSpeed(int16_t mouse, int16_t maximum, int16_t divisor, float speed_max)
{
    const int16_t bounded = mouse > maximum ? maximum : mouse < -maximum ? -maximum
                                                                         : mouse;
    // 保留旧工程整数 /5 截断；速度只在这里换算一次。
    return -static_cast<float>(bounded / divisor) / (maximum / divisor) * speed_max;
}

inline float IntegratePitch(float target, float speed, uint32_t elapsed_ms, uint32_t max_dt_ms, float minimum, float maximum)
{
    const uint32_t dt = elapsed_ms < max_dt_ms ? elapsed_ms : max_dt_ms;
    return std::fmax(minimum, std::fmin(maximum, target + speed * static_cast<float>(dt) * 0.001f));
}

/** 仅识别输入动作；发射达速、热量和卡弹仍由 Shoot 管理。 */
class Class_MouseTrigger
{
  public:
    void Reset()
    {
        require_release = true;
        pressed = false;
    }
    bool Update(bool down, uint32_t now, uint32_t long_press_ms)
    {
        if (require_release)
        {
            if (!down)
                require_release = false;
            return false;
        }
        const bool short_release = pressed && !down && now - start_ms < long_press_ms;
        if (!pressed && down)
            start_ms = now;
        pressed = down;
        return short_release;
    }
    bool Pressed() const
    {
        return pressed;
    }
    bool Burst(uint32_t now, uint32_t long_press_ms) const
    {
        return pressed && now - start_ms >= long_press_ms;
    }

  private:
    bool require_release = true;
    bool pressed = false;
    uint32_t start_ms = 0U;
};
} // namespace KeyboardInput

#endif
