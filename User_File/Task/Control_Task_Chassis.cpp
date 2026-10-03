/**
 * @file Control_Task_Chassis.cpp
 * @brief ChassisBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；周期内不等待 CAN 发送，不解析协议。
 *
 * - 老步兵底盘板（`LEGACY_INFANTRY_CHASSIS`）：输入适配 → RobotCmd → Chassis。
 *   RobotCmd 的三个输出都是本地发布：底盘目标给本板 Chassis，Yaw 速度目标给本板
 *   Chassis 持有的 Yaw 轴，Shoot 命令经 0x065 原始通道转发给云台板，本板无执行器。
 * - 框架四舵轮底盘板：先将板间命令发布到本地 Topic，再更新 Chassis 并产生反馈。
 */
#include "Chassis.h"
#include "Init.h"
#include "cmsis_os2.h"
#if LEGACY_INFANTRY_CHASSIS
#include "RobotCmd.h"
#include "message_center.h"
#include "output.h"
#include "remote_input.h"
#else
#include "board_transport.h"
#endif

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

#if LEGACY_INFANTRY_CHASSIS
    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind()))
    {
        for (;;) osDelay(1000U);
    }
    (void)Chassis_Init();
    /* 遥控接收依赖 UART BSP 与 init_finished，放在设备初始化之后。 */
    (void)RemoteInput_Init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        /* 输入适配先于 RobotCmd，保证本周期发布的命令来自本周期通道。 */
        RemoteInput_Update();
        RobotCmd_Update();
        Chassis_Update();
    }
#else
    BoardTransport_Init();
    (void)Chassis_Init();
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        BoardTransport_Poll();
        Chassis_Update();
    }
#endif
}
