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

/** 保留整数 /5 量化；计数仅在新 UART 帧到达时转换，不乘控制周期。 */
inline float PitchIncrement(int16_t mouse, int16_t maximum, int16_t divisor, float rad_per_count)
{
    const int16_t bounded = mouse > maximum ? maximum : mouse < -maximum ? -maximum
                                                                         : mouse;
    return static_cast<float>((bounded / divisor) * divisor) * rad_per_count;
}

/** 跨板累计计数差分；未就绪/恢复首帧只捕获姿态与计数基准。 */
class Class_MouseYaw
{
  public:
    void Reset()
    {
        captured = false;
        last_delta = 0;
    }
    float Update(uint16_t counts, float actual_rad, bool ready, float rad_per_count, float sign)
    {
        if (!ready || !captured)
        {
            target_rad = actual_rad;
            previous = counts;
            captured = ready;
            last_delta = 0;
        }
        else
        {
            const uint16_t difference = static_cast<uint16_t>(counts - previous);
            last_delta = difference < 0x8000U ? static_cast<int32_t>(difference) : static_cast<int32_t>(difference) - 65536;
            previous = counts;
            target_rad = std::remainder(target_rad + sign * rad_per_count * last_delta, 6.283185307179586f);
        }
        return target_rad;
    }
    int32_t Delta() const
    {
        return last_delta;
    }

  private:
    bool captured = false;
    uint16_t previous = 0U;
    int32_t last_delta = 0;
    float target_rad = 0.0f;
};

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
