#ifndef RM_REMOTE_INPUT_H
#define RM_REMOTE_INPUT_H

/**
 * 初始化 Remote 输入；应在 RobotCmd 初始化之前调用一次。
 *
 * 默认（单板安全模板）：绑定 UART5 S.BUS，失败时输入互锁保持关闭。
 * 老步兵底盘板（`LEGACY_INFANTRY_CHASSIS`）：除 UART5 S.BUS 外，同时绑定
 * BoardConfig 的板间链路总线，用于向云台板下发 0x065/0x070/0x075。
 *
 * ```text
 * UART5 S.BUS → RemoteInput_Update → InputState(Remote) → SourceArbitration → RobotCmd
 *                  |                      ↑
 *                  |- 摇杆整形 / 速度档    |- 底盘速度目标与 Yaw 速度目标
 *                  |- 云台跟随 / 坐标旋转
 *                  └- FDCAN2 0x065/0x070/0x075 → 云台板
 * ```
 *
 * 老步兵底盘板的输入互锁沿用老工程：上电锁定，健康帧连续 200 ms 才解锁，
 * 坏帧或失联立即锁定并转发零通道；未解锁期间提交空输入，由 SourceArbitration
 * 输出安全态（底盘 ZERO_FORCE、云台 DISABLED）。
 */
bool RemoteInput_Init(void);
/** ControlTask 先调用本函数，再运行 RobotCmd_Update。 */
void RemoteInput_Update(void);

/**
 * @brief 读取 S.BUS 遥控链路的在线状态（SBUS Device 内部 Daemon 结果）。
 * @return true 表示链路 liveness 在线；未初始化时返回 false。
 * @note 供诊断层读取 Daemon 结论，不重复实现超时计算；输入健康互锁
 *       （50 ms 帧新鲜度、失控位、回中解锁）仍由 RemoteInput_Update 独立判断。
 */
bool RemoteInput_IsLinkOnline(void);

#endif // RM_REMOTE_INPUT_H
