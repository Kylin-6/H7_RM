/**
 * @file Control_Task_Chassis.cpp
 * @brief ChassisBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；输入适配 → RobotCmd → Chassis。周期内不等待
 *          CAN 发送，不解析协议。RobotCmd 的三个输出都是本地发布：底盘目标给
 *          本板 Chassis，Yaw 速度目标给本板 Chassis 持有的 Yaw 轴，Shoot 命令经
 *          0x065 原始通道转发给云台板，本板无发射执行器。
 */
#include "Chassis.h"
#include "Init.h"
#include "gimbal_imu_transport.h"
#include "cmsis_os2.h"

#include "Diagnostics.h"
#include "RobotCmd.h"
#include "message_center.h"
#include "output.h"
#include "remote_input.h"

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind(), SHOOT != 0))
    {
        Diagnostics_PublishInitFailure();
        for (;;) osDelay(1000U);
    }
    const bool chassis_ready = Chassis_Init();
    /* 遥控接收依赖 UART BSP 与 init_finished，放在设备初始化之后。 */
    const bool remote_ready = RemoteInput_Init();
    const bool imu_link_ready = GimbalImuTransport_Init();
    if (!chassis_ready || !remote_ready || !imu_link_ready)
    {
        Diagnostics_PublishInitFailure();
    }

    uint8_t diagnostic_divider = 0U;
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        /* 输入适配先于 RobotCmd，保证本周期发布的命令来自本周期通道。 */
        GimbalImuTransport_Update();
        RemoteInput_Update();
        RobotCmd_Update();
        Chassis_Update();
        if (++diagnostic_divider >= 10U)
        {
            diagnostic_divider = 0U;
            Diagnostics_Publish();
            Chassis_RecordFlashLog();
        }
    }
}
