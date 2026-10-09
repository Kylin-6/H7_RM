#ifndef RM_REMOTE_INPUT_H
#define RM_REMOTE_INPUT_H

#include <stdint.h>
#include "keyboard_protocol.h"

/**
 * Remote 输入源适配，按板型选择实现（构建期源码选择，同一接口）：
 *
 * - remote_input.cpp（SingleBoard 模板）：UART5 S.BUS → SBUS_Device 解析 →
 *   健康检查/解锁去抖 → 通道映射。
 * - remote_input_forwarded.cpp（GimbalBoard，老步兵云台板）：本模块承担底盘板到
 *   云台板的板间输入适配。0x065 是底盘板代收遥控后转发的关键通道，属于 Remote
 *   输入源：
 *
 * ```text
 * 底盘板 0x065 (FDCAN2) -> Class_ChassisBoard -> RemoteInput_Update()
 *        |- Pitch 通道 --两级低通 + 线性映射--> ControlInput.gimbal
 *        |- 火控开关 -----> ControlInput.shoot（ShootMode）
 *        |- 波轮档位 --线性映射--> ControlInput.shoot.loader_speed_rad_s
 *        └────────────── InputState_SubmitRemote() -> SourceArbitration -> RobotCmd
 * ```
 *
 * 两种实现都在 ControlTask 上下文调用 InputState_SubmitRemote()；链路/帧失效时
 * 提交空输入，由 SourceArbitration 输出 safe state。
 */

/** 初始化 Remote 输入；应在 RobotCmd 初始化之前调用一次。 */
bool RemoteInput_Init(void);
/** ControlTask 先调用本函数，再运行 RobotCmd_Update。 */
void RemoteInput_Update(void);

/**
 * @brief 读取 0x065 板间遥控链路的在线状态（ChassisBoard 内部 Daemon 结果）。
 * @return true 表示链路 liveness 在线；S.BUS 输入实现（SingleBoard）恒返回 false。
 * @note 供诊断层读取 Daemon 结论，不重复实现超时计算；控制输入的时效仍由
 *       RemoteInput_Update 内的通道 freshness 独立判断。
 */
bool RemoteInput_IsLinkOnline(void);

/**
 * @brief 读取最近一帧的三个原始通道值（未滤波、未映射）。
 * @param fire  输出火控开关通道；可为 nullptr。
 * @param dial  输出拨弹盘（波轮）通道；可为 nullptr。
 * @param pitch 输出 Pitch 轴通道；可为 nullptr。
 * @return true 表示三个通道都有 100 ms 内的有效数据。
 * @note 只读访问器，供遥测任务观测链路原始量使用。
 *       S.BUS 输入实现（SingleBoard）恒返回 false 并清零输出。
 */
bool RemoteInput_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch);

/* 老步兵双板：读取 SBUS/0x065 的 CH5 来源选择和健康许可；过期返回 Stop/false。 */
bool RemoteInput_GetReceiverState(ReceiverMode &mode);

#endif // RM_REMOTE_INPUT_H
