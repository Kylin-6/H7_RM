#include "Diagnostics.h"

Struct_Robot_Diagnostic Diagnostics_BuildSnapshot(const Struct_Gimbal_Diagnostic& g,
                                                  const Struct_Shoot_Diagnostic& s, bool remote_valid, bool pitch_enable_timeout)
{
    Struct_Robot_Diagnostic d{};
    if (!g.initialized || !s.initialized)
        d.fault_mask |= DIAG_APP_INIT;
    if (!remote_valid)
        d.fault_mask |= DIAG_REMOTE;
    if (!g.ins_valid)
        d.fault_mask |= DIAG_INS;
    if (g.pitch.fault)
        d.fault_mask |= DIAG_PITCH_FAULT;
    if (g.pitch.required && !g.pitch.online)
        d.fault_mask |= DIAG_PITCH_OFFLINE;
    if (pitch_enable_timeout)
        d.fault_mask |= DIAG_PITCH_ENABLE;
    if (s.jam_failed)
        d.fault_mask |= DIAG_JAM_TIMEOUT;
    if (s.loader.required && !s.loader.online)
        d.fault_mask |= DIAG_LOADER_OFFLINE;
    if (s.left.fault)
        d.fault_mask |= DIAG_LEFT_FAULT;
    if (s.right.fault)
        d.fault_mask |= DIAG_RIGHT_FAULT;
    if (s.left.required && !s.left.online)
        d.fault_mask |= DIAG_LEFT_OFFLINE;
    if (s.right.required && !s.right.online)
        d.fault_mask |= DIAG_RIGHT_OFFLINE;
    d.permitted = g.permitted || s.permitted;
    d.waiting = g.waiting || (s.permitted &&
                              (!s.left.ready || !s.right.ready || !s.loader.ready));
    return d;
}

Struct_Diagnostic_Pattern Diagnostics_SelectPattern(const Struct_Robot_Diagnostic& d,
                                                    bool system_fatal, bool control_fresh, uint32_t uptime_ms, uint32_t& visible_mask)
{
    visible_mask = d.fault_mask;
    if (system_fatal)
        visible_mask |= DIAG_SYSTEM_FATAL;
    if (!control_fresh && uptime_ms >= 3000U)
        visible_mask |= DIAG_CONTROL_STALE;
    uint32_t display_mask = visible_mask;
    if (uptime_ms < 3000U)
        display_mask &= DIAG_SYSTEM_FATAL | DIAG_APP_INIT;
    static constexpr Struct_Diagnostic_Pattern priority[] = {
        {DIAG_SYSTEM_FATAL, 255, 255, 255, 1},
        {DIAG_APP_INIT, 255, 255, 255, 2},
        {DIAG_CONTROL_STALE, 255, 255, 255, 3},
        {DIAG_REMOTE, 255, 255, 0, 1},
        {DIAG_INS, 255, 0, 255, 1},
        {DIAG_PITCH_FAULT, 255, 0, 0, 2},
        {DIAG_PITCH_ENABLE, 255, 0, 0, 3},
        {DIAG_PITCH_OFFLINE, 255, 0, 0, 1},
        {DIAG_JAM_TIMEOUT, 0, 255, 255, 2},
        {DIAG_LOADER_OFFLINE, 0, 255, 255, 1},
        {DIAG_LEFT_FAULT, 255, 128, 0, 3},
        {DIAG_RIGHT_FAULT, 255, 128, 0, 4},
        {DIAG_LEFT_OFFLINE, 255, 128, 0, 1},
        {DIAG_RIGHT_OFFLINE, 255, 128, 0, 2},
    };
    for (const auto& p : priority)
        if ((display_mask & p.selected_fault) != 0U)
            return p;
    if (uptime_ms < 3000U || d.waiting || !control_fresh)
        return {0U, 0, 0, 255, 0, true};
    return d.permitted ? Struct_Diagnostic_Pattern{0U, 0, 255, 0, 0, false}
                       : Struct_Diagnostic_Pattern{0U, 0, 0, 255, 0, false};
}

bool Diagnostics_LightOn(const Struct_Diagnostic_Pattern& p, uint32_t elapsed_ms)
{
    if (p.slow)
        return elapsed_ms % 1000U < 500U;
    if (p.pulses == 0U)
        return true;
    const uint32_t phase = elapsed_ms % (300U * p.pulses + 900U);
    return phase < 300U * p.pulses && phase % 300U < 150U;
}
