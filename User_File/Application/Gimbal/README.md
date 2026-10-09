# 老步兵单 Pitch 云台

本分支的云台 App 按实车只有一个 Pitch 电机编写。Yaw 由底盘板控制，本板不保存
Yaw 电机、PID 或就绪状态；双轴例程的 MotorMit 位置/速度控制路径已移除。
公共 `GimbalCmd` / `GimbalFeedback` 保持框架布局，Yaw 命令字段被忽略，Yaw 反馈仅为姿态观测。

## 所有权与数据流

```text
底盘 0x065 → RemoteInput → InputState/来源仲裁 → RobotCmd → Gimbal_Command_Topic
DM-IMU → DM_IMU_InsBridge → INS_State_Topic → Gimbal_Update → Class_DMMotor → CAN BSP
```

ControlTask 按 1 kHz 调用，反馈每 10 个周期发布一次。Gimbal 的私有 Context 唯一持有
Pitch 电机和外环状态，不向外暴露电机指针。独立 `Application/Pitch` 已合并移除；
原无人调用的手工目标覆盖/S 曲线接口不保留，目标统一通过 RobotCmd 发布。

## 接线与标定

接线在 `User_Config/Board/gimbal_board_config.cpp`；机构参数集中在
[Gimbal_Config.h](Gimbal_Config.h)。

| 项目 | 当前老步兵值 |
| --- | --- |
| Pitch 电机 | FDCAN1，节点 0x09，反馈 0x019，预先配置 MIT 模式 |
| DM-IMU | FDCAN3，请求 0x66，反馈 0x33 |
| 电机协议量程 | ±3.14 rad、±30 rad/s、±10 N·m |
| 姿态目标范围 | -40°～+15°，边界转换为 rad |
| 遥控目标斜坡 | 3 rad/s |
| 最大输出力矩 | 0.5 N·m |
| 输出方向 | IMU 正方向与电机正方向相反，最终力矩乘 -1 |
| 每次使能延迟 | 2000 ms，等待期不发送 Enable 或 MIT 力矩 |
| INS / 电机运动反馈时效 | 100 ms |

输入的两级低通保留老工程 alpha=dt/(tau+dt)，在 Init 时反解框架 IIR 的截止频率。
参数来自原老步兵控制，不作为其他机构默认标定。电源、FDCAN 重传与旧底盘协议配置未改。

## 控制与安全契约

- `IMU`：接收绝对 Pitch 姿态目标，限位后经框架 Slope 从当前姿态平滑接入。
- `LOCK` / `DISABLED`：本单轴应用保持失能，沿用老步兵无明确目标不带力矩启动的行为。
- 位置 PID + 不对称目标速度前馈 - IMU 速度阻尼 + Stribeck 摩擦补偿 + 低带宽扰动估计。
  电机端 MIT `kp/kd` 恒为 0，仅用 `t_ff` 执行力矩。
- 扰动积分不继续推高饱和力矩，允许反向积分退出饱和；非有限力矩计算结果请求失能。
- 扰动补偿上限为 0.15 N·m，总力矩仍限幅 0.5 N·m，用于补偿静止重力偏差。
- 运动时冻结扰动学习并保持补偿，不再按 0.2 s 时间常数衰减支撑力矩；停稳后继续学习。
  输入撤销、INS 失效或电机掉线等停机路径仍清零补偿，恢复从当前姿态重新起步。
- 静止学习增益为 0.8 N·m/(rad·s)，速度阻尼为 0.060 N·m·s/rad，位置刚度为
  0.52 N·m/rad；速度前馈保持正向 0.012、负向 0.018 N·m·s/rad，目标限速保持原值。
  当前机构的采样结果与验证边界见
  [2026-10-04 Pitch 调参记录](../../../sysid/reports/pitch_tuning_2026-10-04.md)。
- DM-IMU 桥沿用欧拉角差分速度：序号差乘标称 1 ms，限幅 ±3 rad/s，变间隔一阶低通
  tau=10 ms，结果放在 INS 的 `gyro_y_rad_s`。它是 Pitch 姿态速度，不是原始机体系陀螺。
- 桥只在收到新欧拉角帧时发布；无数据不发布零姿态续期，Gimbal 根据 Topic 时间戳停机。
- 初始化不发使能、不置零、不写电机持久化参数。注册失败时 Update 安全返回。
- 有效 IMU 命令与新鲜 INS 持续 2 s 后发送使能请求，启动阶段不要求电机先反馈；
  只有新鲜反馈确认电机 ready 后才执行力矩闭环，避免“等反馈才能使能”的启动互锁。
- 输入撤销、INS 过期、闭环运行中电机掉线或电机故障时请求失能并清除目标路径与扰动状态。
  许可恢复重新等待 2 s，电机 ready 后从当前姿态向当前有效目标限速过渡。
- DMMotor 处理协议请求边沿及 100 Hz 失败补交/状态纠正；Gimbal 每 1 ms 提交最新力矩，
  不阻塞等待发送。软件提交成功不代表设备执行成功。

`Gimbal_GetStatus()` 用于观察 CONFIG_ERROR / DISABLE / FAULT / ENABLING / READY。
`GimbalFeedback.enabled` 表示当前控制许可有效且本板唯一 Pitch 电机 ready。

## Pitch 曲线观测

EmberProbe 可只读采样 `Remote_Pitch_Channel`、`Remote_Pitch_Valid` 和
`Gimbal_Debug` 的标量成员：`command_rad` 为遥控滤波后的命令，`target_rad` /
`target_speed_rad_s` 为限速后的闭环目标，`actual_rad` / `actual_speed_rad_s`
为实际 INS 反馈，`command_torque_nm` 为电机协议方向的提交力矩，
`feedback_torque_nm` 为电机报告的反馈力矩。两者不能视为精确测力结果。
检查 `controlling`、`ins_valid`、`motor_ready` 与 `submitted`；提交成功不代表执行确认。
停机时指令力矩显示零，不表示电机已确认失能或机械已停止。

观测值由 ControlTask 更新，不允许写入这些变量调参或控制电机。运行中跨字段读取
非原子快照，`tick_ms` 为本轮更新时间；用相同批次采样比较趋势，不能据此分析单周期因果。
USART1 的原 24 通道发射遥测格式保持不变。调参前保持发射关闭，在机械范围中部做
小幅动作，比较跟踪误差、超调、稳定时间和力矩饱和比例；不能仅凭静止数据修改增益。

## 故障指示灯

本板 WS2812 的颜色、闪烁次数、故障优先级和排查步骤见
[WS2812 debug 指南](../../../docs/debug_ws2812.md)。灯效只观察，不修改控制许可。

## 验证边界

主机回归位于独立测试工作区的 `Tests/InfantryMigration`，编译迁移后的实际应用、DM/DJI
驱动、输入适配和消息中心，检查单轴注册、2 s 延迟、恢复、INS 断线和发射事件。
固件验证覆盖 SingleBoard / GimbalBoard / ChassisBoard。原双轴主机测试不再适用于此单轴 App。
实机尚需核对方向、量程、2 s 时序、断线恢复和 Pitch 阻尼；主机测试不替代上板闭环验证。

键鼠输入与参数边界见 [老步兵键鼠控制](../Input/KEYBOARD_CONTROL.md)；本机构的现有闭环标定保持不变。
