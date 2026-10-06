/**
 * @file Control_Task_Gimbal.cpp
 * @brief GimbalBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；先处理输入适配，再由 RobotCmd 输出本地
 *          云台/发射命令，最后更新 Gimbal/Shoot。周期内不等待
 *          CAN/电机执行，不负责遥控解析或动态路由。
 *          老步兵云台板不使用框架板间 Transport：遥控经底盘板 0x065 帧转发，
 *          由 RemoteInput 输入适配提交，三个 Output 均为本地发布。
 */
#include "Gimbal.h"
#include "RobotCmd.h"
#include "remote_input.h"
#include "Shoot.h"
#include "Init.h"
#include "gimbal_imu_transport.h"
#include "message_center.h"
#include "output.h"
#include "cmsis_os2.h"

#include "dm_imu_ins.h"
#include "Diagnostics.h"
#include "stm32h7xx_hal.h"

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

    /*
     * 老步兵云台板：该板不控底盘，不使用框架板间 Transport 下发 ChassisCmd；
     * RobotCmd 的三个 Output 全部绑定本地 Topic。发射应用随构建编入，
     * 向 RobotCmd_Init 显式传入可用性。
     */
    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind(), true))
    {
        Diagnostics_PublishInitFailure();
        for (;;) osDelay(1000U);
    }
    (void)RemoteInput_Init();
    if (!GimbalImuTransport_Init())
    {
        Diagnostics_PublishInitFailure();
    }
    /* BMI088 缺席：DM-IMU 统一发布姿态，Gimbal 只读取 INS Topic。 */
    (void)DM_IMU_InsBridge_Init();
    (void) Gimbal_Init();
    (void)Shoot_Init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        /* 输入适配先于 RobotCmd，保证本周期发布的命令来自本周期通道。 */
        RemoteInput_Update();
        DM_IMU_InsBridge_Update();
        GimbalImuTransport_Update();
        RobotCmd_Update();
        Gimbal_Update();
        Shoot_Update();
        static uint32_t last_diagnostic_ms = 0U;
        const uint32_t now_ms = HAL_GetTick();
        if (now_ms - last_diagnostic_ms >= 10U)
        {
            last_diagnostic_ms = now_ms;
            Diagnostics_Publish();
        }
    }
}
