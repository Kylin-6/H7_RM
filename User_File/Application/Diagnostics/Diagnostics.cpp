#include "Diagnostics.h"

Struct_Chassis_Diagnostic Diagnostics_BuildSnapshot(const Struct_Chassis_Diagnostic_Input &g,
                                                    bool remote_valid, const bool enable_timeout[5])
{
    Struct_Chassis_Diagnostic d{};
    if (!g.initialized) d.fault_mask |= 1U << 1;
    if (!remote_valid) d.fault_mask |= 1U << 3;
    if (!g.ins_valid) d.fault_mask |= 1U << 4;
    d.permitted = g.permitted;
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        const auto &m = g.motor[i];
        d.waiting = d.waiting || (m.requested_enabled && !m.ready);
        if (m.feedback.state > 1U) d.fault_mask |= 1U << (i < 4U ? 5U + i : 17U);
        if (m.requested_enabled && !m.online) d.fault_mask |= 1U << (i < 4U ? 9U + i : 18U);
        if (enable_timeout[i]) d.fault_mask |= 1U << (i < 4U ? 13U + i : 19U);
    }
    return d;
}

Struct_Diagnostic_Pattern Diagnostics_SelectPattern(const Struct_Chassis_Diagnostic &d,
                                                    bool system_fatal, bool control_fresh,
                                                    uint32_t uptime_ms, uint32_t &visible_mask)
{
    uint32_t mask = d.fault_mask;
    if (system_fatal) mask |= 1U;
    if (!control_fresh) mask |= 1U << 2;
    visible_mask = mask;
    // 开机宽限只显示初始化失败，其他异常暂用蓝色慢闪。
    const uint32_t visible = uptime_ms < 3000U ? mask & 3U : mask;
    uint8_t red = 0U, green = 0U, blue = 255U, pulses = 0U;
    uint32_t selected = 0U;
    bool slow = uptime_ms < 3000U || d.waiting;
    // 类别优先级：系统、应用、控制、遥控、INS、轮故障、轮使能、轮离线、Yaw。
    constexpr uint8_t priority[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                    13, 14, 15, 16, 9, 10, 11, 12, 17, 19, 18};
    for (uint8_t bit : priority)
    {
        if (!(visible & (1U << bit))) continue;
        selected = 1U << bit;
        slow = false;
        if (bit <= 2U) { red = green = blue = 255U; pulses = bit + 1U; }
        else if (bit == 3U) { red = green = 255U; blue = 0U; pulses = 1U; }
        else if (bit == 4U) { red = 255U; blue = 255U; pulses = 1U; }
        else if (bit <= 8U) { red = 255U; blue = 0U; pulses = bit - 4U; }
        else if (bit <= 12U) { red = 255U; green = 128U; blue = 0U; pulses = bit - 8U; }
        else if (bit <= 16U) { red = green = 255U; blue = 0U; pulses = bit - 11U; }
        else { green = blue = 255U; pulses = bit == 17U ? 2U : (bit == 18U ? 1U : 3U); }
        break;
    }
    if (!selected && !slow && d.manual_protection) { red = 128U; green = 0U; blue = 255U; }
    else if (!selected && !slow && d.permitted) { green = 255U; blue = 0U; }
    return {selected, red, green, blue, pulses, slow};
}

bool Diagnostics_LightOn(const Struct_Diagnostic_Pattern &p, uint32_t elapsed_ms)
{
    if (p.slow) return elapsed_ms % 1000U < 500U;
    if (!p.pulses) return true;
    const uint32_t burst_ms = p.pulses * 300U;
    const uint32_t phase = elapsed_ms % (burst_ms + 900U);
    return phase < burst_ms && phase % 300U < 150U;
}
