#ifndef RM_SOURCE_ARBITRATION_H
#define RM_SOURCE_ARBITRATION_H

#include "input_state.h"

struct InputDecision
{
    ChassisCmd chassis{};
    GimbalCmd gimbal{};
    ShootCmd shoot{};
    // 单发/三连发为离散动作；序号在来源内递增，重复读取不重复射击。
    ShootEvent shoot_event{};
    uint32_t shoot_event_sequence = 0U;
    InputSource source = InputSource::Remote;
    bool armed = false;
};

/** 固定优先级：运行许可 → 按设置要求 Remote 健康 → 新鲜所选来源 → 可选 Vision。 */
InputDecision SourceArbitration_Resolve(const InputState &state);

#endif
