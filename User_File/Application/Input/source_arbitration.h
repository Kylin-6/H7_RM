#ifndef RM_SOURCE_ARBITRATION_H
#define RM_SOURCE_ARBITRATION_H

#include "input_state.h"

struct InputDecision
{
    ChassisCmd chassis{};
    GimbalCmd gimbal{};
    ShootCmd shoot{};
    InputSource source = InputSource::Remote;
    bool armed = false;
};

/** 固定优先级：运行许可 → 按设置要求 Remote 健康 → 新鲜所选来源 → 可选 Vision。 */
InputDecision SourceArbitration_Resolve(const InputState &state);

#endif
