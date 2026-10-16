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
    E = 1U << 7,
    F = 1U << 9
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

/** 完整计数仅在新 UART 帧到达时转换，不乘控制周期，不丢弃细瞄增量。 */
inline float PitchIncrement(int16_t mouse, int16_t maximum, float rad_per_count)
{
    const int16_t bounded = mouse > maximum ? maximum : mouse < -maximum ? -maximum
                                                                         : mouse;
    return static_cast<float>(bounded) * rad_per_count;
}

/** Pitch 每帧累加角度，以实际帧间隔估计速度前馈；控制周期不重复消费计数。 */
class Class_MousePitch
{
  public:
    void Reset(float actual, float minimum, float maximum)
    {
        target = std::fmax(minimum, std::fmin(maximum, actual));
        received = false;
        interval_delta = velocity = 0.0f;
        period_ms = 33U; // VT02 标称 30 Hz；首帧及异常间隔仅用标称周期估计前馈。
    }
    float Update(int16_t mouse, uint32_t timestamp, int16_t maximum, float rad_per_count,
                 float minimum, float upper, float speed_limit)
    {
        const float next = std::fmax(minimum, std::fmin(upper, target + PitchIncrement(mouse, maximum, rad_per_count)));
        const float delta = next - target;
        target = next;
        const uint32_t elapsed = timestamp - last_ms;
        if (!received || elapsed > 50U)
            period_ms = 33U;
        else if (elapsed != 0U)
            period_ms = elapsed;
        // 同一 DMA chunk 的帧可能具有相同毫秒时间戳；累加而不是除以零或覆盖。
        if (delta == 0.0f || !received || elapsed != 0U)
            interval_delta = delta;
        else
            interval_delta += delta;
        const float estimate = interval_delta / (0.001f * static_cast<float>(period_ms));
        velocity = std::fmax(-speed_limit, std::fmin(speed_limit, estimate));
        last_ms = timestamp;
        received = true;
        return delta;
    }
    float Target() const
    {
        return target;
    }
    float Velocity(uint32_t now) const
    {
        // 无后续帧时只保留一个估计帧周期的前馈；角度目标仍保持，失联保护由 Input 仲裁。
        return received && now - last_ms <= period_ms ? velocity : 0.0f;
    }

  private:
    float target = 0.0f;
    float interval_delta = 0.0f;
    float velocity = 0.0f;
    uint32_t last_ms = 0U;
    uint32_t period_ms = 33U;
    bool received = false;
};

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
