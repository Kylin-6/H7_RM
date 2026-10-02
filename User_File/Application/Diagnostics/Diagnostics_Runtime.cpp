#include "Diagnostics.h"
#include "Gimbal.h"
#include "Init.h"
#include "Shoot.h"
#include "board_config.h"
#include "bsp_ws2812.h"
#include "input_state.h"
#include "message_center.h"
#include "stm32h7xx_hal.h"

Struct_Diagnostic_LED_State Diagnostics_LED_State;

void Diagnostics_PublishInitFailure()
{
    Struct_Robot_Diagnostic d{};
    d.fault_mask = DIAG_APP_INIT;
    MessageCenter::Robot_Diagnostic_Topic.Publish(d);
}

void Diagnostics_Publish()
{
    static bool enable_pending = false;
    static uint32_t enable_start_ms = 0U;
    const uint32_t now = HAL_GetTick();
    const auto g = Gimbal_GetDiagnostic();
    const auto s = Shoot_GetDiagnostic();
    const auto input = InputState_Read();
    const bool remote_valid = input.remote.valid && now - input.remote.received_ms <= 100U;
    // 连续请求期间只观察超时，不重发或修改使能请求。
    if (g.pitch.requested_enabled && !g.pitch.ready)
    {
        if (!enable_pending)
        {
            enable_pending = true;
            enable_start_ms = now;
        }
    }
    else
        enable_pending = false;
    const auto d = Diagnostics_BuildSnapshot(g, s, remote_valid,
                                             enable_pending && now - enable_start_ms >= 1000U);
    MessageCenter::Robot_Diagnostic_Topic.Publish(d);
}

void Diagnostics_LED_Update()
{
    if (!BoardConfig_Get().indicators)
        return;
    static Struct_Diagnostic_Pattern previous{};
    static uint32_t pattern_start_ms = 0U;
    const uint32_t now = HAL_GetTick();
    Struct_Robot_Diagnostic d{};
    // 即使过期也保留最后故障位图，同时由白灯指示控制停止。
    const auto snapshot = MessageCenter::Robot_Diagnostic_Topic.ReadWithMeta();
    if (snapshot.valid)
        d = snapshot.data;
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    const bool control_fresh = snapshot.valid && now_us >= snapshot.timestamp_us &&
                               now_us - snapshot.timestamp_us <= 50000U;
    const auto p = Diagnostics_SelectPattern(d, System_Init_GetState() == SYSTEM_INIT_FATAL,
                                             control_fresh, now, Diagnostics_LED_State.fault_mask);
    if (p.selected_fault != previous.selected_fault || p.red != previous.red ||
        p.green != previous.green || p.blue != previous.blue || p.slow != previous.slow)
    {
        pattern_start_ms = now;
        previous = p;
    }
    Diagnostics_LED_State.pattern = p;
    const bool on = Diagnostics_LightOn(p, now - pattern_start_ms);
    // 15% 亮度；颜色设置和 SPI 刷新都在同一 TIM_1ms_Task 中。
    BSP_WS2812.Set_RGB(on ? p.red * 15U / 100U : 0U,
                       on ? p.green * 15U / 100U : 0U,
                       on ? p.blue * 15U / 100U : 0U);
    BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback();
}
