#ifndef RM_REMOTE_INPUT_H
#define RM_REMOTE_INPUT_H

#include <stdint.h>

/**
 * 默认配置：在 RobotCmd_Init 后绑定 UART5 S.BUS，失败时输入互锁保持关闭。
 *
 * 老步兵云台板配置（`LEGACY_INFANTRY_GIMBAL`）：本模块承担底盘板到云台板的
 * 板间输入适配。0x065 是底盘板代收遥控后转发的关键通道，属于 Remote 输入源：
 *
 * ```text
 * 底盘板 0x065 (FDCAN2) -> Class_ChassisBoard -> RemoteInput_Update()
 *        |- Pitch 通道 --两级低通 + 线性映射--> ControlInput.gimbal
 *        |- 火控开关 -----> ControlInput.shoot（ShootMode）
 *        |- 波轮档位 --线性映射--> ControlInput.shoot.loader_speed_rad_s
 *        └────────────── InputState_SubmitRemote() -> SourceArbitration -> RobotCmd
 * ```
 *
 * 链路超过 100 ms 没有新帧时提交空输入，由 SourceArbitration 输出 safe state：
 * 云台 DISABLED、发射 OFF。
 */

/** 初始化 Remote 输入；应在 RobotCmd 初始化之前调用一次。 */
bool RemoteInput_Init(void);
/** ControlTask 先调用本函数，再运行 RobotCmd_Update。 */
void RemoteInput_Update(void);

/**
 * @brief 读取最近一帧的三个原始通道值（未滤波、未映射）。
 * @param fire  输出火控开关通道；可为 nullptr。
 * @param dial  输出拨弹盘（波轮）通道；可为 nullptr。
 * @param pitch 输出 Pitch 轴通道；可为 nullptr。
 * @return true 表示三个通道都有 100 ms 内的有效数据。
 * @note 只读访问器，供遥测任务观测链路原始量使用。
 *       非 legacy 配置（S.BUS 输入）恒返回 false 并清零输出。
 */
bool RemoteInput_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch);

#endif // RM_REMOTE_INPUT_H
