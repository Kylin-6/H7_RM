# 老步兵底盘板应用

ChassisBoard 固件承载老步兵底盘板：四路 DM 麦轮、一路挂在本板的 Yaw DM 电机、
SBUS 遥控接收，并经 FDCAN2 向云台板下发 0x065/0x070/0x075 下行帧。云台 Pitch 与
发射机构在云台板固件上，不在本模块内。

同一份 `Chassis.cpp` 还保留框架四舵轮 AGV 实现（`CHASSIS`），当前默认预设不编译；
两套实现的编译开关互斥，见 [Application 指南](../README.md)。

## 配置与来源

机构与控制参数集中在 [Chassis_Config.h](Chassis_Config.h) 的
`LegacyInfantryChassisConfig`；总线归属由 `User_Config/Board/chassis_board_config.cpp`
提供。数值全部沿用老工程（`rm/demo` 的 `APP/ChassisTask.c`、`APP/GimbalTask.c`、
`User/bsp/bsp_def.h`）与老步兵测试分支的实车版本。

| 项目 | 取值 | 来源与限制 |
| --- | --- | --- |
| 底盘四轮总线 / ID / 反馈 ID | FDCAN1，0x50~0x53 / 0x60~0x63 | 与 demo 的 `bsp_CAN.c` 一致；速度模式 |
| 底盘电机量程 | ±3.14 rad、±200 rad/s、±10 N·m | DM3519 协议量程，须与电机端 PMAX/VMAX/TMAX 相同 |
| Yaw 总线 / ID / 反馈 ID | FDCAN1，0x03 / 0x05 | Yaw 电机挂在本板，MIT 模式 |
| Yaw 电机量程 | ±3.14 rad、±30 rad/s、±10 N·m | 同上 |
| 摇杆 Yaw 速度上限 | 15 rad/s | 与老工程一致，须按机构复核 |
| Yaw MIT Kp / Kd | 0 / 1.4（中心）、1.0（推进）、1.6（反向） | Kp 恒为 0，即纯速度 + 阻尼控制 |
| Yaw 力矩前馈 | 增益 0.002、限幅 0.35 N·m、变化率 8 N·m/s | 由规划加速度换算 |
| 三轴速度上限 | 30 / 30 / 50（抽象量纲） | 实车未标定真实 m/s，见「量纲约定」 |
| 麦轮单轮限幅 | 30 | 与三轴同量纲 |
| 云台跟随 | 正前方 Yaw = π rad，比例增益 8 | 机械安装与操作手感的实车值 |
| 控制路径周期 | 2 ms（1 kHz 调度 2 分频） | 老工程周期；同时把 DM 速度帧压回 FDCAN1 带宽 |

## 控制契约

`ChassisCmd` 由 RobotCmd 每 10 ms 刷新，本模块按 100 ms 时效读取，Topic 过期即按
`ZERO_FORCE` 处理。`GimbalCmd`（Yaw 轴）沿用框架云台命令的语义：只在目标变化或安全
撤销时发布，因此按最新值读取、不判时效，否则摇杆保持不动会被误判为过期；遥控失联时
RobotCmd 会把命令撤销为 `DISABLED`。

- 底盘三轴：输入层的抽象速度目标 → `*_FromSi` 还原量纲 → 非对称速率规划 → 麦轮组合
  `w0 = y + x + w`、`w1 = y - x + w`、`w2 = -y - x + w`、`w3 = x - y + w` → 单轮限幅 →
  `SetSpeed()`。组合式不做轮距与半径换算，与老工程一致。
- Yaw：摇杆速度（±15 rad/s）减去底盘自转角速度前馈（BMI088 的 `gyro_z`，10 ms 新鲜
  度）→ 非对称速率规划（加速度上限随摇杆比例在 60~150 之间插值）→ 自适应阻尼与
  力矩前馈（逐周期限速）→ `SetMIT(0, speed, 0, kd, torque_ff)`。
- 三轴与 Yaw 的速率规划用框架 `Class_Slope` 执行斜坡，非对称速率策略（反向先刹停、
  松手按释放减速度、加速/减速分别限幅、零速吸附）留在应用层，数值与老工程
  `SpeedPlanning_UpdateRateLimited` 逐项对应。

### 量纲约定

老步兵底盘三轴速度沿用实车验证过的抽象量纲（与麦轮预混后的 DM 轮速同量纲），没有可信
的 m/s 标定。输入仲裁按框架 SI 边界（`INPUT_MAX_TRANSLATION_M_S` /
`INPUT_MAX_ROTATION_RAD_S`）校验，因此：Input 侧用 `LegacyChassis_*_ToSi` 归一化，
本模块用 `LegacyChassis_*_FromSi` 还原，两者共用 `Chassis_Config.h` 中的同一份比例。
这是边界换算约定，不代表这些数值已经标定为 SI 物理量。

反馈按同一约定回写规划值（本板不测量真实车体速度）：`online` 表示四轮在线，
`enabled` 表示四轮 ready 且当前不是 `ZERO_FORCE`。Yaw 轴状态按 `GimbalFeedback`
发布（`yaw_rad` 为电机单圈角，`yaw_speed_rad_s` 为反馈速度，`ins_valid` 为 INS 新鲜），
输入层用它做平移坐标旋转与跟随判断，本板不拥有 Pitch。

## 安全与恢复

- 上电默认失能：`Chassis_Init()` 先下发 Yaw 安全目标，再对全部已配置电机
  `RequestEnabled(false)`；即使部分电机注册失败也尝试停住它们。
- 使能请求是边沿语义，由框架 `Class_DMMotor` 与 100 Hz `StatusTask::ServiceAll`
  保证补发；应用不手写周期重发。
- 任一 Topic 过期、遥控失联或未解锁时，RobotCmd 立即发布 `ZERO_FORCE` 与 `DISABLED`，
  四轮与 Yaw 同周期停手。
- 输入层在链路失效或未解锁时向云台板转发零通道，避免云台板继续使用旧摇杆值。
- 武装指示灯：红=未解锁/失能，蓝=已解锁；状态跳变时补一次即时 WS2812 刷新。

## 板间下行帧

三个帧都经框架 CAN BSP 的周期通道下发（同一 (总线, ID) 只保留最新值），
每 2 ms 刷新一次，与老工程 `GimbalTask` 周期一致。

| 帧 | 总线 | 布局 |
| --- | --- | --- |
| 0x065 | FDCAN2 | byte 0~1 火控开关、2~3 发射速度、4~5 Pitch 轴，大端 int16，已减中位；最小有效长度 6 |
| 0x070 | FDCAN2 | byte 0~1 Yaw 电机角度、2~3 地面系 Yaw，大端 int16，单位 0.01°，`(deg - 180) * 100` |
| 0x075 | FDCAN2 | byte 0~1 枪管热量上限、2~3 冷却值、byte 4 机器人 ID；裁判系统未接入时全 0 |

- 火控开关极性：实车与云台板约定相反，输入层转发前取反。
- 地面系 Yaw 没有独立陀螺仪来源，按老工程约定暂用 Yaw 电机角度代替。
- 云台板当前只解析 0x065；0x070/0x075 保持与老工程逐帧一致，便于后续接入。
- 链路单向下行、无应答，本板无法感知云台板是否收到。

## 与老工程（rm/demo）核对

逐项对照老工程 `APP/ChassisTask.c`、`APP/GimbalTask.c`、`User/bsp/bsp_CAN.c`、
`User/bsp/bsp_def.h`、`User/bsp/sbus_channel_bsp.c`、`User/algorithm/CHASSIS_ALG.c`、
`User/algorithm/SpeedPlanning.c`、`User/module/DM_drv.{c,h}`，以及云台板老工程
`H7_RM` 的 0x065 解码端。

已核对一致：

| 项目 | 老工程 | 本实现 |
| --- | --- | --- |
| 底盘四轮 | FDCAN1，SPEED 模式，ID 0x50~0x53，反馈 0x60~0x63 | 同（BoardConfig + Chassis_Config） |
| Yaw | FDCAN1，MIT 模式，ID 0x03，反馈 0x05 | 同 |
| 协议量程 | 底盘 3519：±3.14 rad / ±200 rad/s / ±10 N·m；Yaw：±3.14 / ±30 / ±10 | 同 |
| 麦轮组合 | `y+x+w, y-x+w, -y-x+w, x-y+w`，整轮限幅 30 | 同 |
| 速率规划 | 非对称加速度/减速度/释放/反向 + 零速门限 0.1，dt 2 ms | 策略同，斜坡改由 `Class_Slope` 执行 |
| 通道映射 | 跟随=CH5(sbus[4])、前后=CH2(sbus[1])、左右=CH1(sbus[0])、旋转=CH10(sbus[9])、档位=CH7(sbus[6])、Yaw=CH4(sbus[3]) | 同 |
| 摇杆整形与档位 | 死区 0.04/0.03、指数 0.35/0.30/0.55、±784 幅值、档位映射 30/30/50 | 同 |
| 跟随 | 正前方 π rad、比例增益 8、限幅取档位旋转上限 | 同 |
| Yaw 控制律 | 摇杆取反、±15 rad/s、`gyro_z` 前馈增益 1、加速度 60↔150 插值、减速 120/释放 75/反向 250、Kp=0、Kd 1.4↔1.0（反向 1.6）限速 5、力矩前馈 0.002/±0.35/限速 8 | 同 |
| 使能帧 | 7×0xFF + 0xFC/0xFD，MIT 发 ID、SPEED 发 ID\|0x200 | 框架驱动一致 |
| 控制帧 | MIT 位宽 16/12/12/12/12；SPEED 为 4 字节小端 float | 框架驱动一致 |
| 0x065 | FDCAN2，大端 int16：sbus[5] / sbus[8] / sbus[2]，最小 6 字节 | 同（云台板老工程解码端一致） |
| 0x070/0x075 | FDCAN2，ID 与字节布局同上 | 同 |
| 互锁与指示灯 | 上电锁定、健康 200 ms 解锁；红灯失能、蓝灯武装；SBUS 健康=无 frame-lost/failsafe 且帧新鲜 | 同 |

有意偏差（原因随列）：

- **火控开关取反**：老工程原样透传；云台板固件（老步兵云台分支）按 ≤-500 判为按下，
  而实车按下时原始通道为正，因此发送端必须取反，见输入层 `kFireSwitchInvert`。更换
  遥控器或改通道极性时必须与云台板一起复核。
- **失联立即失能**：老工程允许失联 200 ms 后才失能；本实现坏帧/失联即锁定（更严格）。
- **失联主动转发零通道**：老工程在拿不到 SBUS 帧时干脆不发 0x065；本实现按 2 ms 继续
  转发零通道，明确清掉云台板侧的旧摇杆值。
- **Yaw 反馈有效性门控**：老工程无条件用电机位置做坐标旋转；本实现要求 Yaw 轴反馈
  有效，避免反馈失效时算出接近 180° 的偏差把速度方向翻转（老工程已修过的实车故障）。
- **使能生命周期**：老工程在解锁时补发一次使能；本实现交给框架驱动的边沿请求与
  StatusTask 补交。
- **失能期间规划回零**：老工程继续维护规划状态、只靠发送门关闭输出；本实现让规划按
  释放减速度回零，重新使能时从零速起步。
- **控制量被反馈门控**：框架 `SetSpeed`/`SetMIT` 在电机未 ready（离线/未使能/故障）时
  只发零目标。老工程底盘四轮**从未解析过反馈**（`DM_Chassis_4Motor_Get` 无调用者，
  FDCAN1 回调只处理 Yaw 电机），属于开环下发；本实现因此新引入了“四轮反馈必须正常”
  的依赖，见下方待上板确认项。
- **0x070 编码**：老工程把弧度值直接减 180 再乘 100（rad/deg 混用）；本实现按
  rad → degree 在编码边界转换一次并夹到 int16 值域（与老步兵测试分支的修正一致）。
- **未迁移**：串口调试/JustFloat 遥测（老工程 `DebugTask`）、UART7 维特陀螺仪（老工程
  只用于调试打印）都不属于控制链，本板不接；0x070 的地面系 Yaw 仍是待接入项。

## 验证

- `cmake --preset ChassisBoard` 与本仓库三树（SingleBoard / GimbalBoard / ChassisBoard）
  从零构建通过，无新增告警。
- 未做实机验证：电机方向与 ID、Yaw 力矩方向与阻尼、麦轮正方向、云台跟随方向与增益、
  板间帧收发、失联失能时序都需要上板核对。
- 需上板确认的新依赖：四路底盘电机的反馈必须真正到达 FDCAN1 的 0x60~0x63（框架在
  反馈不可用时只发零目标），否则表现为“电机在线但轮子不动”；老工程没有这条依赖。
- 数值上未做与老工程的逐周期对拍（`Class_Slope` 斜坡替代 `SpeedPlanning` 的策略与
  系数逐项一致，但缺少主机回归证据）。
