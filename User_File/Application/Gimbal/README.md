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
- 遥控：位置 PID + 不对称目标速度前馈 - IMU 速度阻尼 + Stribeck 摩擦补偿 + 低带宽扰动估计。
- 键鼠：位置外环＋速度内环，完整鼠标计数生成角目标，按帧间隔估计速度前馈；初值与验证边界见
  [键鼠 Pitch 细瞄与串级控制](../Input/KEYBOARD_CONTROL.md#键鼠-pitch-细瞄与串级控制)。
  电机端 MIT `kp/kd` 恒为 0，仅用 `t_ff` 执行力矩。
- 扰动积分不继续推高饱和力矩，允许反向积分退出饱和；非有限力矩计算结果请求失能。
- 扰动补偿上限为 0.15 N·m，总力矩仍限幅 0.5 N·m，用于补偿静止重力偏差。
- 运动时冻结扰动学习并保持补偿，不再按 0.2 s 时间常数衰减支撑力矩；停稳后继续学习。
  输入撤销、INS 失效或电机掉线等停机路径仍清零补偿，恢复从当前姿态重新起步。
- 静止学习增益为 0.8 N·m/(rad·s)，速度阻尼候选为 0.090 N·m·s/rad，位置刚度候选为
  0.78 N·m/rad；速度前馈保持正向 0.012、负向 0.018 N·m·s/rad，目标限速保持原值。
  首版 0.65/0.075 用户反馈仍晃，本轮在其基础上再提高 20%，尚需实机加减速对比；此前机构的采样结果与验证边界见
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
`position_error_rad` 为限速目标与 INS 的差值；`unclamped_torque_nm` 为最终总限幅前的
合力矩（IMU 正方向，PID 内部限幅仍生效）；`torque_saturated` 表示合力矩超出 ±0.5 N·m。
`torque_saturation_count` 累计饱和控制周期，停机保留、计满保持；其余新增控制值停机清零。
这组统计只反映最终合力矩限幅，不能推断电机实际电流或机械输出能力。
检查 `controlling`、`ins_valid`、`motor_ready` 与 `submitted`；提交成功不代表执行确认。
停机时指令力矩显示零，不表示电机已确认失能或机械已停止。

观测值由 ControlTask 更新，不允许写入这些变量调参或控制电机。运行中跨字段读取
非原子快照，`tick_ms` 为本轮更新时间；用相同批次采样比较趋势，不能据此分析单周期因果。
USART1 的原 24 通道发射遥测格式保持不变。调参前保持发射关闭，在机械范围中部做
小幅动作，比较跟踪误差、超调、稳定时间和力矩饱和比例；不能仅凭静止数据修改增益。

## 故障指示灯

本板 WS2812 的颜色、闪烁次数、故障优先级和排查步骤见
[WS2812 debug 指南](../../../docs/debug_ws2812.md)。灯效只观察，不修改控制许可。

## 增刚验证记录（2026-10-11）

首版提高到 0.65/0.075 后，用户反馈仍会晃动；本轮改为 0.78/0.090，增幅 20%。
沿用上一版诊断，不修改 IMU 采集、测速或原生 gyro 路径。
`/tmp/h7-pitch-stiffness-v2/run.py` 编译实际 Gimbal/PID/Slope/Input/RobotCmd，2728 项主机检查通过，
ASan/UBSan 检查通过；覆盖两种模式增益、保持/制动、总力矩限幅、饱和累计、积分防饱和、
停机清零及恢复等待。只构建本目录 GimbalBoard，未烧录、未进行加减速实机验收。
构建有既存 PID 类型声明警告；本轮未修改该类型。

实机应在相同负载和中部 Pitch 角度对比起步、刹车的峰值偏差、恢复时间、饱和次数及外部画面。
出现持续振荡或峰值增大时回退增益；不能以主机测试通过作为抗扰效果已改善的结论。

## 历史验证边界

主机回归位于独立测试工作区的 `Tests/InfantryMigration`，编译迁移后的实际应用、DM/DJI
驱动、输入适配和消息中心，检查单轴注册、2 s 延迟、恢复、INS 断线和发射事件。
固件验证覆盖 SingleBoard / GimbalBoard / ChassisBoard。原双轴主机测试不再适用于此单轴 App。
实机尚需核对方向、量程、2 s 时序、断线恢复和 Pitch 阻尼；主机测试不替代上板闭环验证。

键鼠输入与参数边界见 [老步兵键鼠控制](../Input/KEYBOARD_CONTROL.md)；增刚记录对应历史候选；当前额外加速度采集与前馈见下节。


## 加速度计 Pitch 前馈（待标定）

DM-IMU 安装方向由用户确认：X 朝枪口、Y 朝左、Z 朝上。保留欧拉角请求和现有测速，
每 5 个 1 ms 周期额外请求一次 0x01 加速度寄存器。沿用老工程 `DMIMU.c` 的 CAN 映射：
DATA[2..7] 为 XYZ 小端 uint16，线性范围 ±235.2 m/s²，包含重力。
Device 保存序号和本地微秒接收时间，短临界区读取一致快照；加速度帧不能刷新欧拉角在线状态。
请求提交失败不补发，累计在 `DM_IMU_Accel_Request_Failures`；200 Hz 请求及应答增加 CAN 负载，须实机检查应答间隔。

`pitch_accel_feedforward.h` 按 ZYX 右手系姿态扣除机体系重力，逐轴限幅 ±6 m/s²，
对新样本使用实际接收间隔作 20 ms 一阶低通。补偿力矩为
`mass_height_kg_m * a_x - mass_forward_kg_m * a_z`，分别对应质量乘重心高度和前向距离。
力矩按 IMU +Y 方向加入主控，再沿用已有电机方向转换；独立限幅 ±0.05 N·m、变化率 1 N·m/s，
最终合力矩仍限制 ±0.5 N·m。加速度或姿态超过 20 ms、时间异常或非有限值时撤销前馈；
恢复首帧只建立基准，停机清除滤波和前馈。启用后 X/Z 加速度达到 0.2 m/s² 时冻结静态扰动学习。
不改变 INS 或板间字段语义，不修改原有姿态闭环、输入许可和失联停机。

**质量和重心未知，两项系数默认均为零，当前固件只增加观测，不产生加速度补偿力矩。**
参数集中在 `Gimbal_Config.h` 的 `pitch_accel`。启用前须在多个静止 Pitch 角度核对原始比力、
欧拉角正方向和扣重力残差：静止时残差应接近零，不能只按外壳轴向推断欧拉角符号。
再在中部角度、相同负载、关闭发射条件下记录起步/刹车，标定两个带符号系数并核对补偿方向。
IMU 随 Pitch 转动，其离轴旋转加速度也包含在测量内，本版未分离该项；不要用快速转动过程直接标定底盘扰动。
没有实机标定或外部画面对比，不能宣称抗晃改善。

`Gimbal_Debug.accel_raw_m_s2` / `accel_linear_m_s2` / `accel_filtered_m_s2` 分别为原始比力、
扣重力残差、限幅滤波值；`accel_age_us`、`accel_sequence`、`accel_fresh` 用于确认样本时效，
`accel_ff_enabled` 表示数据新鲜且系数非零，`accel_ff_torque_nm` 为最终总限幅前的前馈分量。
这些字段只是观测，不能当作方向已经验证或电机实际输出证明。

本轮 `/tmp/h7-pitch-accel/run.py` 对实际应用执行 2795 项 ASan/UBSan 主机检查，覆盖默认关闭、
重力扣除、新样本滤波、力矩符号/限幅/变化率、20 ms 时效和恢复，以及原输入/安全回归。
另以 `/tmp/h7-pitch-accel/device-test.py` 编译实际 Device/bridge，验证小端量程、接收时间与序号、
中断状态恢复、加速度不续期姿态、请求调度及失败计数饱和。
仅构建 GimbalBoard：Flash 142384 B、DTCM 117088 B（89.33%）；构建保留既存 PID/volatile 警告。
尚未进行传感器量程、CAN 实际负载、补偿方向和抗扰效果的实机验收。


### 首次实机读取发现的发送瓶颈

2026-10-11 只读采样中，加速度请求失败计数与 CAN 队列满计数均为 55520，
设备加速度快照 `sequence=0`、`valid=false`；同时姿态仍有效。
旧发送任务每条总线每周期只发送一帧，无法消化 FDCAN3 每秒 1200 帧请求。
现将云台分支 FDCAN3 的固定预算调整为每周期最多两帧，FDCAN1/2 保持一帧；
硬件拒绝时保留帧并退出，不阻塞重试。尚需更新固件后确认加速度序号增长和失败计数变化，
这些旧固件读数尚不能用于前馈标定；主机测试没有替代这项实机检查。

修复验证：`/tmp/h7-can-budget-test.py` 提取实际 `BSP_CAN_SendAsync`，ASan/UBSan 下
检查 20 万周期、两种任务先后顺序、1200 Hz 生产吞吐、分总线预算、硬件拒绝保留及顺序重试。
仅构建 GimbalBoard，Flash 142408 B、DTCM 117088 B；未烧录修复，未取得有效加速度或标定系数。
