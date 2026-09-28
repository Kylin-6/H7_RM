/**
 * @file StatusTask.cpp
 * @brief 低频设备在线状态检查任务。
 * @details Low 优先级、100 Hz，osDelayUntil 阻塞等待；输入是设备 Feed 时间戳和
 *          DM 待恢复标记，输出是在线跃迁检查及入队失败的限频重试。不解析 CAN，
 *          不负责整车安全策略或电机控制算法。装有 DM 电机的固件同时在此显示
 *          BMI088 通信故障灯效。
 */

#include "daemon.h"
#include "cmsis_os2.h"
#include "Init.h"
#include "bsp_bmi088.h"
#include "bsp_ws2812.h"
#include "message_center.h"
#include "sys_timestamp.h"
#if H7_HAS_DM_MOTOR
#include "dmmotor.h"
#endif

static constexpr uint32_t BMI088_INS_TIMEOUT_MS = 100U;
static constexpr uint32_t BMI088_BLINK_PERIOD_MS = 1000U;

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    // 使用绝对唤醒时间，避免 CheckAll() 执行时间累积到任务周期中。
    uint32_t next_wake_tick = osKernelGetTickCount();
    const uint32_t start_tick = next_wake_tick;
    uint32_t fault_start_tick = next_wake_tick;
    bool fault_active = false;

    for (;;)
    {
        // 100 Hz 足以覆盖当前最短 100 ms 设备超时，并远低于 1 kHz 控制频率。
        DaemonManager::CheckAll();
#if H7_HAS_DM_MOTOR
        Class_DMMotor::ServiceAll();
#endif

        const uint32_t now_tick = osKernelGetTickCount();
        INS_State ins_state = {};
        const bool ins_fresh = MessageCenter::INS_State_Topic.ReadFresh(
            ins_state, BMI088_INS_TIMEOUT_MS * 1000ULL);
        const uint64_t accel_update_us =
            BSP_BMI088.Get_Accel_Last_Update_Timestamp_Us();
        const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
        const bool accel_stale = accel_update_us == 0U ||
            now_us < accel_update_us ||
            now_us - accel_update_us > BMI088_INS_TIMEOUT_MS * 1000ULL;
        const bool bmi088_fault =
            (System_Init_GetFailureMask() & SYSTEM_INIT_FAILURE_BMI088) != 0U ||
            (now_tick - start_tick >= BMI088_INS_TIMEOUT_MS &&
             (!ins_fresh || accel_stale));

        if (bmi088_fault)
        {
            if (!fault_active)
            {
                fault_start_tick = now_tick;
                fault_active = true;
            }
            const uint32_t phase_ms =
                (now_tick - fault_start_tick) % BMI088_BLINK_PERIOD_MS;
            // 每秒两次紫色短闪：0-100 ms、200-300 ms，其余时间熄灭。
            BSP_WS2812.Set_Override_Color(
                (phase_ms < 100U || (phase_ms >= 200U && phase_ms < 300U))
                    ? WS2812_COLOR_MAGENTA : WS2812_COLOR_BLACK);
        }
        else if (fault_active)
        {
            BSP_WS2812.Clear_Override_Color();
            fault_active = false;
        }

        next_wake_tick += 10U;
        osDelayUntil(next_wake_tick);
    }
}
