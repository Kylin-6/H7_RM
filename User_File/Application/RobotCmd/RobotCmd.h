#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "message_types.h"

/** 装载机器人安全启动默认命令并发布一次；云台/发射命令后续由输入适配模块直发 Topic。 */
void RobotCmd_Init(void);
/** 周期读取模块反馈，并发布发生变化的底盘控制命令。 */
void RobotCmd_Update(void);

/** 底盘命令仍经此中转；云台/发射命令请直接发布对应 Topic。 */
void RobotCmd_SetChassis(const ChassisCmd &command);
/** 将一次性射击动作压入固定容量 FIFO；队列已满时返回 false。 */
bool RobotCmd_PushShootEvent(const ShootEvent &event);

/** 返回 false 表示对应应用尚未发布过有效反馈，输出对象保持不变。 */
bool RobotCmd_GetGimbalFeedback(GimbalFeedback &feedback);
bool RobotCmd_GetChassisFeedback(ChassisFeedback &feedback);
bool RobotCmd_GetShootFeedback(ShootFeedback &feedback);

#endif
