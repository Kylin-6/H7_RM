/**
 * @file Com.h
 * @brief 上层输入适配（老步兵配置）：SBUS 遥控接收与云台板链路转发。
 * @details
 * 位于 RobotCmd 之前，是“遥控 → 应用命令”的唯一入口。按 Application/README.md 的
 * 契约，它只调用 RobotCmd_SetGimbal / RobotCmd_SetChassis 更新目标，不直接发布任何
 * Message Center 通道，也不触碰电机。
 *
 * 原 rm/demo 中散落在 TransmitTask 与 SafetyTask 的职责在此合并：
 * - TransmitTask 的 SBUS 取帧与通道映射
 * - SafetyTask 的遥控健康互锁（健康 200 ms 解锁，失联 200 ms 锁定）
 * 板间链路的 0x065/0x070/0x075 转发顺序也保持与 demo 的 GimbalTask 一致。
 */

#ifndef COM_H
#define COM_H

#include <stdbool.h>

/**
 * @brief 初始化遥控接收与云台板链路。
 * @note 必须在 System_Init() 之后调用：UART BSP 只在 init_finished 置位后才向
 *       接收回调分发数据。初始化失败不阻塞其他 Application。
 * @return true 表示 SBUS 与板间链路均已就绪。
 */
bool Communication_Init(void);

/**
 * @brief 遥控输入周期入口，由 Control_Task 在 RobotCmd_Update() 之前调用。
 * @details 读取最近一帧 SBUS、维护健康互锁、转发板间状态帧，并把摇杆输入整形为
 *          ChassisCmd 与 GimbalCmd 交给 RobotCmd。
 */
void Communication_Update(void);

#endif // COM_H
