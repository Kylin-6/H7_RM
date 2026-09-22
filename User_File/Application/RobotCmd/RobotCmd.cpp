/**
 * @file RobotCmd.cpp
 * @brief 机器人命令分发与反馈聚合，参考 Meta-Embedded-NG 设计。
 * @details
 * 云台/发射命令由输入适配模块（Communication）直接发布到 Topic；底盘命令仍经
 * 这里的 setter 中转。RobotCmd 负责安全启动命令的一次性发布与各模块反馈聚合，
 * 不直接访问电机、CAN 或 IMU 设备。
 */

#include "RobotCmd.h"

#include "message_center.h"

static Publisher<GimbalCmd> Gimbal_Command_Publisher(
    MessageCenter::Gimbal_Command_Topic);
static Publisher<ChassisCmd> Chassis_Command_Publisher(
    MessageCenter::Chassis_Command_Topic);
static Publisher<ShootCmd> Shoot_Command_Publisher(
    MessageCenter::Shoot_Command_Topic);
static Subscriber<GimbalFeedback> Gimbal_Feedback_Subscriber(
    MessageCenter::Gimbal_Feedback_Topic);
static Subscriber<ChassisFeedback> Chassis_Feedback_Subscriber(
    MessageCenter::Chassis_Feedback_Topic);
static Subscriber<ShootFeedback> Shoot_Feedback_Subscriber(
    MessageCenter::Shoot_Feedback_Topic);

/*
 * 命令发布者归属：云台/发射命令由输入适配模块（Communication）直接发布到
 * Topic；底盘命令仍经这里的 setter 中转。RobotCmd 只保留安全启动命令的
 * 一次性发布与反馈聚合，不再缓存云台/发射命令。
 */
static ChassisCmd Chassis_Command;
static GimbalFeedback Gimbal_Feedback;
static ChassisFeedback Chassis_Feedback;
static ShootFeedback Shoot_Feedback;

/* Dirty 标志避免没有变化时重复发布命令。 */
static bool Chassis_Command_Dirty;
static bool Gimbal_Feedback_Valid;
static bool Chassis_Feedback_Valid;
static bool Shoot_Feedback_Valid;
static uint8_t RobotCmd_Feedback_Divider;

void RobotCmd_Init(void)
{
    Chassis_Command = {};
    /*
     * RM 安全启动默认值：云台保持 Gimbal_Init 捕获的姿态（LOCK），发射机构
     * 保持关闭。直接发布一次；后续周期由输入适配模块接管命令发布。
     */
    GimbalCmd gimbal_safe_command;
    gimbal_safe_command.mode = GimbalMode::LOCK;
    Gimbal_Command_Publisher.Publish(gimbal_safe_command);
    Shoot_Command_Publisher.Publish(ShootCmd{});
    Chassis_Command_Dirty = true;
    Gimbal_Feedback_Valid = false;
    Chassis_Feedback_Valid = false;
    Shoot_Feedback_Valid = false;
    RobotCmd_Feedback_Divider = 0U;
}

void RobotCmd_Update(void)
{
    /* 应用反馈以 100 Hz 拉取，首次未收到反馈时 Valid 保持 false。 */
    if (RobotCmd_Feedback_Divider == 0U)
    {
        GimbalFeedback gimbal_feedback;
        ChassisFeedback chassis_feedback;
        ShootFeedback shoot_feedback;

        if (Gimbal_Feedback_Subscriber.Read(gimbal_feedback))
        {
            Gimbal_Feedback = gimbal_feedback;
            Gimbal_Feedback_Valid = true;
        }
        if (Chassis_Feedback_Subscriber.Read(chassis_feedback))
        {
            Chassis_Feedback = chassis_feedback;
            Chassis_Feedback_Valid = true;
        }
        if (Shoot_Feedback_Subscriber.Read(shoot_feedback))
        {
            Shoot_Feedback = shoot_feedback;
            Shoot_Feedback_Valid = true;
        }
    }
    RobotCmd_Feedback_Divider = (RobotCmd_Feedback_Divider + 1U) % 10U;

    /* 云台/发射命令由输入适配模块直接发布；这里只发布底盘目标。 */
    if (Chassis_Command_Dirty)
    {
        Chassis_Command_Publisher.Publish(Chassis_Command);
        Chassis_Command_Dirty = false;
    }
}

void RobotCmd_SetChassis(const ChassisCmd &command)
{
    Chassis_Command = command;
    Chassis_Command_Dirty = true;
}

bool RobotCmd_PushShootEvent(const ShootEvent &event)
{
    return MessageCenter::Shoot_Event_Queue.Push(event);
}

bool RobotCmd_GetGimbalFeedback(GimbalFeedback &feedback)
{
    if (!Gimbal_Feedback_Valid)
    {
        return false;
    }
    feedback = Gimbal_Feedback;
    return true;
}

bool RobotCmd_GetChassisFeedback(ChassisFeedback &feedback)
{
    if (!Chassis_Feedback_Valid)
    {
        return false;
    }
    feedback = Chassis_Feedback;
    return true;
}

bool RobotCmd_GetShootFeedback(ShootFeedback &feedback)
{
    if (!Shoot_Feedback_Valid)
    {
        return false;
    }
    feedback = Shoot_Feedback;
    return true;
}
