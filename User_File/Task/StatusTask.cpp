/**
 * @file StatusTask.cpp
 * @brief 低频设备在线状态检查任务。
 * @details Low 优先级、100 Hz，osDelayUntil 阻塞等待；输入是设备 Feed 时间戳和
 *          设备请求状态，输出是在线跃迁检查及 100 Hz 设备安全/协议服务。不解析 CAN，
 *          不负责整车安全策略或电机控制算法。
 */

#include "daemon.h"
#include "cmsis_os2.h"
#include "dji_motor.h"
#if CHASSIS
#include "sbus.h"
#include "Chassis.h"
#include <cmath>
#include "SEGGER_RTT.h"
#include <cstdio>
#endif
#if H7_HAS_DM_MOTOR
#include "dmmotor.h"
#endif

#if CHASSIS
namespace
{
// 定点文本避免 printf 浮点支持依赖；非有限值以 valid/active 字段判定。
long Milli(float value)
{
    if (!std::isfinite(value)) return 0L;
    const float scaled = value * 1000.0f;
    return static_cast<long>(std::fmax(-2147483000.0f, std::fmin(2147483000.0f, scaled)));
}
}
#endif

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    // 使用绝对唤醒时间，避免 CheckAll() 执行时间累积到任务周期中。
    uint32_t next_wake_tick = osKernelGetTickCount();
#if CHASSIS
    uint8_t remote_divider = 0U;
    uint8_t yaw_divider = 0U;
    uint32_t yaw_dropped = 0U;
#endif

    for (;;)
    {
        // 100 Hz 统一记录跃迁；实时控制的新鲜度检查不等待此任务。
        DaemonManager::CheckAll();
        Class_DJIMotor::ServiceAll();
#if H7_HAS_DM_MOTOR
        Class_DMMotor::ServiceAll();
#endif
#if CHASSIS
        // 50 Hz 观察最新控制快照；缓冲满丢弃整行，绝不等待主机。
        if (++yaw_divider >= 2U)
        {
            yaw_divider = 0U;
            Struct_Yaw_Tuning sample{};
            const bool fresh = Chassis_ReadYawTuning(sample);
            char line[384];
            const int length = std::snprintf(line, sizeof(line),
                "YAW ms=%lu valid=%u active=%u target=%ld actual=%ld err=%ld stick=%ld pid=%ld base=%ld ff=%ld cmd=%ld motor=%ld kd=%ld torque=%ld drop=%lu\r\n",
                static_cast<unsigned long>(HAL_GetTick()),
                fresh && sample.imu_valid ? 1U : 0U, fresh && sample.active ? 1U : 0U,
                Milli(sample.target_rad), Milli(sample.actual_rad), Milli(sample.error_rad),
                Milli(sample.stick_rad_s), Milli(sample.pid_rad_s), Milli(sample.base_rad_s),
                Milli(sample.ff_rad_s), Milli(sample.command_rad_s), Milli(sample.motor_rad_s),
                Milli(sample.kd), Milli(sample.torque_nm), static_cast<unsigned long>(yaw_dropped));
            if (length <= 0 || static_cast<unsigned>(length) >= sizeof(line) ||
                SEGGER_RTT_Write(0U, line, static_cast<unsigned>(length)) != static_cast<unsigned>(length))
            {
                ++yaw_dropped;
            }
        }
        /* 20 Hz、RTT 通道 0 默认 NO_BLOCK_SKIP；不在控制任务或中断中输出。 */
        if (++remote_divider >= 5U)
        {
            remote_divider = 0U;
            Struct_SBUS_Frame frame{};
            const bool valid = SBUS_ReadLatest(&frame);
            char line[224];
            const int length = std::snprintf(
                line, sizeof(line),
                "RC valid=%u age_ms=%lu lost=%u failsafe=%u CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d CH6=%d CH7=%d CH8=%d CH9=%d CH10=%d\r\n",
                valid ? 1U : 0U,
                valid ? static_cast<unsigned long>(HAL_GetTick() - frame.timestamp_ms) : 0UL,
                static_cast<unsigned>(frame.frame_lost), static_cast<unsigned>(frame.failsafe),
                frame.channels[0], frame.channels[1], frame.channels[2], frame.channels[3],
                frame.channels[4], frame.channels[5], frame.channels[6], frame.channels[7],
                frame.channels[8], frame.channels[9]);
            if (length > 0 && static_cast<unsigned>(length) < sizeof(line))
            {
                (void)SEGGER_RTT_Write(0U, line, static_cast<unsigned>(length));
            }
        }
#endif
        next_wake_tick += 10U;
        osDelayUntil(next_wake_tick);
    }
}
