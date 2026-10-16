#ifndef KEYBOARD_MODES_H
#define KEYBOARD_MODES_H

#include "keyboard_logic.h"

namespace KeyboardInput
{
/** 云台逐帧识别点按；故障保留跟随选择，但取消自转并等待各键释放。 */
class Class_KeyboardModes
{
  public:
    void Reset(bool new_session)
    {
        if (new_session)
            follow = true;
        SetSpin(0U);
        previous = 0U;
        blocked = F | Q | E;
    }

    void Update(uint16_t keys)
    {
        if (Combination(keys))
        {
            SetSpin(0U);
            blocked = F | Q | E;
            previous = keys;
            return;
        }
        blocked &= keys;
        const uint16_t pressed = keys & ~previous & ~blocked;
        previous = keys;
        if ((keys & (Q | E)) == (Q | E))
        {
            SetSpin(0U);
            blocked |= Q | E;
            return;
        }
        if ((pressed & F) != 0U)
            follow = !follow;
        if ((pressed & (Q | E)) != 0U)
        {
            const uint8_t direction = (pressed & Q) != 0U ? 1U : 2U;
            SetSpin(spin == direction ? 0U : direction);
        }
    }

    bool follow = true;
    uint8_t spin = 0U;
    uint8_t revision = 0U;

  private:
    void SetSpin(uint8_t next)
    {
        if (next != spin)
            revision = (revision + 1U) & 31U;
        spin = next;
    }
    uint16_t previous = 0U;
    uint16_t blocked = F | Q | E;
};

/** 底盘就绪恢复只建立基准；F 和重复状态帧均不能重新启动旧自转。 */
class Class_KeyboardSpinGuard
{
  public:
    uint8_t Update(const Struct_Keyboard_Frame& frame, bool ready, bool reset)
    {
        if (!ready || reset || !captured)
        {
            spin = 0U;
            revision = frame.spin_revision;
            captured = ready;
        }
        else
        {
            if (frame.spin == 0U || frame.spin_revision != revision)
                spin = frame.spin;
            revision = frame.spin_revision;
        }
        return spin;
    }

  private:
    bool captured = false;
    uint8_t revision = 0U;
    uint8_t spin = 0U;
};
} // namespace KeyboardInput
#endif
