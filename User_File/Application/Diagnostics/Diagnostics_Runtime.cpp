#include "Diagnostics.h"
#include "Init.h"
#include "board_config.h"
#include "bsp_ws2812.h"
#include "remote_input.h"
#include "message_center.h"

Struct_Diagnostic_LED_State Diagnostics_LED_State{};
namespace { bool init_failed = false; }

void Diagnostics_PublishInitFailure(void)
{
    init_failed = true;
    Struct_Chassis_Diagnostic d{};
    d.fault_mask = 1U << 1;
    MessageCenter::Chassis_Diagnostic_Topic.Publish(d);
}

void Diagnostics_Publish(void)
{
    static bool pending[5]{};
    static uint32_t started_ms[5]{};
    bool timeout[5]{};
    const uint32_t now = HAL_GetTick();
    const auto g = Chassis_GetDiagnostic();
    /* DIAG_REMOTE 只消费 S.BUS 链路 Daemon 的在线结果，不再自行做
     * received_ms + timeout 计算；输入健康互锁由 RemoteInput/仲裁负责。 */
    const bool remote_valid = RemoteInput_IsLinkOnline();
    // 连续请求期间只观察超时，不重发或修改使能请求。
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        const auto &m = g.motor[i];
        if (m.requested_enabled && !m.ready)
        {
            if (!pending[i]) started_ms[i] = now;
            pending[i] = true;
        }
        else pending[i] = false;
        timeout[i] = pending[i] && now - started_ms[i] >= 1000U;
    }
    auto d = Diagnostics_BuildSnapshot(g, remote_valid, timeout);
    if (init_failed) d.fault_mask |= 1U << 1;
    MessageCenter::Chassis_Diagnostic_Topic.Publish(d);
}

void Diagnostics_LED_Update(void)
{
    if (!BoardConfig_Get().indicators) return;
    static Struct_Diagnostic_Pattern previous{};
    static uint32_t start_ms = 0U;
    const uint32_t now = HAL_GetTick();
    const auto snapshot = MessageCenter::Chassis_Diagnostic_Topic.ReadWithMeta();
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    const bool fresh = snapshot.valid && now_us >= snapshot.timestamp_us &&
                       now_us - snapshot.timestamp_us <= 50000U;
    const auto p = Diagnostics_SelectPattern(snapshot.data,
        System_Init_GetState() == SYSTEM_INIT_FATAL, fresh, now, Diagnostics_LED_State.fault_mask);
    if (p.selected_fault != previous.selected_fault || p.red != previous.red ||
        p.green != previous.green || p.blue != previous.blue || p.slow != previous.slow)
    {
        previous = p;
        start_ms = now;
    }
    Diagnostics_LED_State.pattern = p;
    const bool on = Diagnostics_LightOn(p, now - start_ms);
    BSP_WS2812.Set_RGB(on ? p.red * 15U / 100U : 0U,
                       on ? p.green * 15U / 100U : 0U,
                       on ? p.blue * 15U / 100U : 0U);
    BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback();
}
