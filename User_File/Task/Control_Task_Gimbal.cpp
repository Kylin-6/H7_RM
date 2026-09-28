/**
 * @file Control_Task_Gimbal.cpp
 * @brief GimbalBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；先处理板间反馈，再由 RobotCmd 输出本地云台/发射和
 *          远端底盘命令，最后更新 Gimbal/Shoot。周期内不等待 CAN/电机执行，
 *          不负责遥控/视觉解析或动态路由。
 */
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Com.h"
#include "Shoot.h"
#include "Init.h"
#include "message_center.h"
#include "output.h"
#include "cmsis_os2.h"

#if LEGACY_INFANTRY_GIMBAL
#include "Com.h"
#else
#include "board_transport.h"
#include "remote_publisher.h"
#endif

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

#if LEGACY_INFANTRY_GIMBAL
    /*
     * 老步兵云台板输入适配：遥控与反馈通道由底盘板经 FDCAN2 的 0x065 帧转发，
     * 由 Communication 模块接收并经 RobotCmd setter 提交。该板不控底盘，
     * 不使用框架板间 Transport 下发 ChassisCmd。
     */
    Communication_Init();
    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
#else
    BoardTransport_Init();
    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static RemotePublisher<ChassisCmd> chassis_output;
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
#endif
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind()))
    {
        for (;;) osDelay(1000U);
    }
    (void)Communication_Init();
    Gimbal_Init();
    (void)Shoot_Init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
#if LEGACY_INFANTRY_GIMBAL
        /* 输入适配先于 RobotCmd，保证本周期发布的命令来自本周期通道。 */
        Communication_Update();
#else
        BoardTransport_Poll();
        Communication_Update();
#endif
        RobotCmd_Update();
        Gimbal_Update();
        Shoot_Update();
    }
}
