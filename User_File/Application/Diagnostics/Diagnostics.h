#ifndef ROBOT_DIAGNOSTICS_H
#define ROBOT_DIAGNOSTICS_H
#include "message_types.h"

// 位图保留所有异常；显示顺序由 Diagnostics_SelectPattern 固定。
enum Enum_Diagnostic_Fault : uint32_t
{
    DIAG_SYSTEM_FATAL = 1U << 0,
    DIAG_APP_INIT = 1U << 1,
    DIAG_CONTROL_STALE = 1U << 2,
    DIAG_REMOTE = 1U << 3,
    DIAG_INS = 1U << 4,
    DIAG_PITCH_FAULT = 1U << 5,
    DIAG_PITCH_OFFLINE = 1U << 6,
    DIAG_PITCH_ENABLE = 1U << 7,
    DIAG_JAM_TIMEOUT = 1U << 8,
    DIAG_LOADER_OFFLINE = 1U << 9,
    DIAG_LEFT_FAULT = 1U << 10,
    DIAG_RIGHT_FAULT = 1U << 11,
    DIAG_LEFT_OFFLINE = 1U << 12,
    DIAG_RIGHT_OFFLINE = 1U << 13,
};
struct Struct_Diagnostic_Pattern
{
    uint32_t selected_fault = 0U;
    uint8_t red = 0U, green = 0U, blue = 0U;
    uint8_t pulses = 0U;
    bool slow = false;
};
struct Struct_Diagnostic_LED_State
{
    uint32_t fault_mask = 0U;
    Struct_Diagnostic_Pattern pattern{};
};
/** TIM_1ms_Task 唯一写入；供调试器观察，不作为控制输入。 */
extern Struct_Diagnostic_LED_State Diagnostics_LED_State;

Struct_Robot_Diagnostic Diagnostics_BuildSnapshot(const Struct_Gimbal_Diagnostic& gimbal,
                                                  const Struct_Shoot_Diagnostic& shoot, bool remote_valid, bool pitch_enable_timeout);
Struct_Diagnostic_Pattern Diagnostics_SelectPattern(const Struct_Robot_Diagnostic& snapshot,
                                                    bool system_fatal, bool control_fresh, uint32_t uptime_ms, uint32_t& visible_mask);
bool Diagnostics_LightOn(const Struct_Diagnostic_Pattern& pattern, uint32_t elapsed_ms);
/** ControlTask：10 ms 周期采集应用诊断。 */
void Diagnostics_Publish();
/** 初始化入口失败时发布白灯故障，不参与控制恢复。 */
void Diagnostics_PublishInitFailure();
/** TIM_1ms_Task：选择灯效并调用既有 WS2812 刷新，唯一颜色写入者。 */
void Diagnostics_LED_Update();
#endif
