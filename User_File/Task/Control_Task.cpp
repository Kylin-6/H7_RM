/**
 * @file Control_Task.cpp
 * @brief 应用层统一控制任务。
 * @details
 * SingleBoard：High1，阻塞等待 1 ms 线程标志；输入为本地 Topic/应用命令，
 * 输出为各应用所属设备目标和反馈。RobotCmd 先发布，再运行 Gimbal/Chassis/Shoot。
 * 周期内不得等待 I/O；本任务不解析协议、不承担板间 CAN 传输。
 */

#include "Chassis.h"
#include "Com.h"
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Com.h"
#include "Shoot.h"
#include "Init.h"
#include "message_center.h"
#include "output.h"
#include "sys_imu.h"
#include "user_task.h"

extern "C" void Control_Task(void* argument)
{
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;)
        {
            osDelay(1000U);
        }
    }

    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind()))
    {
        for (;;) osDelay(1000U);
    }
#if GIMBAL || LEGACY_INFANTRY
    Gimbal_Init();
#endif
    (void)Chassis_Init();
    (void)Shoot_Init();
    /* 遥控接收依赖 UART BSP 与 init_finished，放在各 Device 初始化之后。 */
    (void)Communication_Init();
    // Balance_init();

    for (;;)
    {
        /* 由 1 ms 定时回调唤醒；阻塞等待期间不占用 CPU。 */
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        /* 输入侧先把遥控整形为 InputState，再由命令所有者统一仲裁发布。 */
        Communication_Update();
        /* 命令所有者先发布最新目标，再由各 Application 消费并执行。 */
        RobotCmd_Update();
        System_IMU_Publish_Wit_Fallback();
        Gimbal_Update();
        Chassis_Update();
        Shoot_Update();
        // Balance_loop();
    }
}
