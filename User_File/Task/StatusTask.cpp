/** @file StatusTask.cpp
    @author zzm
    @brief 20 Hz 健康状态任务 */
#include "cmsis_os2.h"
#include "sys_health.h"

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    Sys_Health_Init();
    uint32_t period = (uint32_t)(((uint64_t)osKernelGetTickFreq() * SYS_HEALTH_PERIOD_MS + 999) / 1000);
    if (period == 0)
        period = 1;
    uint32_t deadline = osKernelGetTickCount();
    for (;;)
    {
        Sys_Health_Update();
        deadline += period;
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - deadline) >= 0)
            deadline = now + period;
        osDelayUntil(deadline);
    }
}
