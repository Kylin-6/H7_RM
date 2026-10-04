# 老步兵云台 WS2812 debug 指南

本指南对应 GimbalBoard 的单 Pitch 实车配置。
单颗板载 WS2812 按优先级显示故障，所有异常同时保存在诊断位图中。
指示灯只观察，不修改电机控制、供电、安全互锁或故障复位条件。

## 在线状态来源与 freshness

各周期数据源的 Online / Offline（liveness）唯一由 Device 内的 Daemon 判定
（详见 `User_File/System/Daemon/README.md`）：Pitch / 摩擦轮为 DMMotor Daemon，
拨弹盘为 DJImotor Daemon，DM-IMU 为欧拉角链路 Daemon，0x065 遥控转发为
ChassisBoard Daemon。Diagnostics 只消费这些 Device / Application 诊断快照并映射
为故障码，不自行计算 `last_rx + timeout`，WS2812 只负责显示。

与 Online 相互独立保留的数据 freshness（"这份数据是否可用于当前控制"）：
INS Topic `ReadFresh`、DM-IMU 桥 100 ms 姿态时效、0x065 通道 100 ms 接收时间戳、
电机微秒反馈时间戳和 Shoot / Gimbal 自有业务超时。因此可能出现
"Daemon 仍在线但旧姿态已被 Gimbal 拒绝"或"严格 freshness 先于 Daemon 超时失效"。

## 硬件与固件

| 设备 | 当前连接与参数 |
| --- | --- |
| 板载 WS2812 | SPI6，单颗灯，RGB 接口、GRB 线序，亮度 15% |
| 底盘遥控转发 | FDCAN2，接收 ID 0x065，数据时效 100 ms |
| DM-IMU | FDCAN3，请求 0x66、反馈 0x33，INS 时效 100 ms |
| Pitch | FDCAN1，节点 0x09、反馈 0x019，MIT 模式 |
| 左摩擦轮 | FDCAN1，节点 0x07、反馈 0x027，速度模式 |
| 右摩擦轮 | FDCAN1，节点 0x08、反馈 0x028，速度模式 |
| M2006 / C610 拨弹盘 | FDCAN2，电机 ID 1、反馈 0x201，减速比 36；输出轴直连 7 弹位 |

本板不监控底盘拥有的 Yaw；BMI088、ADC 已禁用，Flash 未安装，均不报警。
电机使用既有外部供电，本功能不改变两路 24V 关闭、仅开板载 5V 的配置。

在工程根目录构建 `cmake --build --preset GimbalBoard`，烧录
`build/GimbalBoard/H7_BSP.elf`。不要选择同目录历史 `GimbalBoard.elf`，
也不要选择 SingleBoard 或验证目录的 ELF。烧录成功不等于程序已运行，烧录后应复位。

## 看灯判断

下表由上到下是类别优先级；一次只显示最高优先级故障，修复后自动显示下一项。
每次亮 150 ms、灭 150 ms；一组结束后再灭 900 ms，然后重复。
例如红色两闪表示 Pitch 上报故障，青色两闪表示卡弹回退超时。

| 灯色 | 闪烁次数 / 含义 | 优先检查 |
| --- | --- | --- |
| 白 | 1：系统初始化致命失败 | `System_Init_GetFailureMask()`；TIM4 / TIM5 等初始化结果 |
| 白 | 2：Gimbal / Shoot 或命令入口初始化失败 | 应用诊断 initialized、设备注册、BoardConfig 和配置量程 |
| 白 | 3：控制诊断超过 50 ms 未更新或从未发布 | ControlTask 是否运行、1 ms 线程标志、任务栈与阻塞情况 |
| 黄 | 1：0x065 遥控转发链路离线（ChassisBoard Daemon） | 底盘是否上电并转发、FDCAN2 接线、波特率和接收 ID |
| 品红 | 1：DM-IMU 无新鲜有效姿态（链路见 DM-IMU Daemon，时效见 INS freshness） | FDCAN3、IMU 供电、0x66 请求与 0x33 欧拉角反馈、INS 时间戳 |
| 红 | 1：Pitch 反馈离线（DMMotor Daemon） | 外部供电、FDCAN1、节点 0x09、反馈 0x019、MIT 模式 |
| 红 | 2：Pitch 电机上报故障 | 达妙反馈 state；按电机实际故障码检查温度、电压和负载 |
| 红 | 3：请求使能后连续 1 s 未确认 ready | Enable 是否发送、电机模式、反馈 state；这只是观察超时，不改变重试策略 |
| 青 | 1：拨弹盘反馈离线（DJImotor Daemon） | C610 外部供电、FDCAN2、ID 1 / 0x201、反馈新鲜度 |
| 青 | 2：卡弹回退超时锁存 | 拨弹盘机械阻塞、编码器实际回退角、方向和电流；不可持续顶弹 |
| 橙 | 1：左摩擦轮离线（DMMotor Daemon） | FDCAN1、供电、节点 0x07 / 反馈 0x027 |
| 橙 | 2：右摩擦轮离线（DMMotor Daemon） | FDCAN1、供电、节点 0x08 / 反馈 0x028 |
| 橙 | 3：左摩擦轮上报故障 | 左轮反馈 state、温度、电压和机械负载 |
| 橙 | 4：右摩擦轮上报故障 | 右轮反馈 state、温度、电压和机械负载 |
| 绿常亮 | 许可有效、姿态及所需设备状态正常 | 仍需遥控给目标，绿灯不表示电机一定正在转动 |
| 蓝慢闪 | 启动或使能等待，亮 / 灭各 500 ms | 开机宽限 3 s；Pitch 每次使能前原有等待 2 s |
| 蓝常亮 | 主动撤销控制许可，且无其他应显示的故障 | 当前 Gimbal / Shoot 命令和来源仲裁 |

同类别多项同时出现时：Pitch 为“硬件故障 → 使能超时 → 离线”；
拨弹盘为“回退超时 → 离线”；摩擦轮为“左故障 → 右故障 → 左离线 → 右离线”。
初始化故障始终优先。开机前 3 s 只显示系统 / 应用初始化失败，其余先蓝色慢闪，
完整位图仍保留已观察到的异常。

电机离线仅在应用要求该电机工作时报警；主动失能不因电机不反馈而报离线。
ShootMode::ON 是机构许可，不等于摩擦轮正在旋转，本分支许可有效时要求三台电机在线。
Pitch 在使能等待完成、准备请求工作后才要求反馈。电机最近上报的 state > 1 会保留报警，
即使随后反馈过期；收到非故障状态后清除。通信离线只说明缺少反馈，不能单凭灯确定
是供电、CAN 接线、ID、模式还是设备本身的问题。

正常卡弹回退不报警。回退半个弹位，编码器到位容差 2°，200 ms 是未到位超时上限。
超时锁存后停止拨弹，撤销发射许可 OFF 才复位；恢复后旧排队动作不补射。
松扳机不等于 ShootMode::OFF，不能仅靠松扳机解除此锁存。

## 调试器查看全部故障

`Diagnostics_LED_State` 由 TIM_1ms_Task 更新：

- `fault_mask`：全部异常，包含系统致命失败和控制快照过期，启动宽限期间也可查看。
- `pattern.selected_fault`：当前灯实际显示的故障位；0 表示正常 / 等待 / 撤销许可。
- `pattern.pulses`：一组闪烁次数；`pattern.slow`：蓝色慢闪标志。
- `pattern.red/green/blue`：未缩放 RGB；实际发送亮度为其 15%。

`MessageCenter::Robot_Diagnostic_Topic.ReadWithMeta()` 是一致快照入口，
数据含 `fault_mask / permitted / waiting`，元信息含有效标志、序号与微秒时间戳。
不要直接从另一任务调用 Gimbal/Shoot 诊断 getter；它们只供 ControlTask 使用。

| 位 / 十六进制 | 异常 |
| --- | --- |
| 0 / 0x0001 | 系统致命失败 |
| 1 / 0x0002 | 应用初始化失败 |
| 2 / 0x0004 | 控制诊断过期 |
| 3 / 0x0008 | 遥控转发无效 |
| 4 / 0x0010 | INS 无效 |
| 5 / 0x0020 | Pitch 硬件故障 |
| 6 / 0x0040 | Pitch 离线 |
| 7 / 0x0080 | Pitch 使能超时 |
| 8 / 0x0100 | 卡弹回退超时 |
| 9 / 0x0200 | 拨弹盘离线 |
| 10 / 0x0400 | 左摩擦轮故障 |
| 11 / 0x0800 | 右摩擦轮故障 |
| 12 / 0x1000 | 左摩擦轮离线 |
| 13 / 0x2000 | 右摩擦轮离线 |

低优先级故障不因未显示而被清除。控制任务停止后保留最后一份故障位图，并叠加白色三闪。
现有 USART1 JustFloat 布局和 CAN 协议不改变。

## 刷新与故障边界

ControlTask 在应用更新后每 10 ms 发布一次快照；TIM_1ms_Task 的已有 10 ms 回调
读取快照、选择灯效、设置 RGB、执行原 SPI6 刷新。同一任务负责颜色设置和发送，
不新增线程，不直接读取其他任务的应用 Context。

GimbalBoard 构建（Diagnostics 模块编入）且 BoardConfig.indicators=true 才运行本灯效。
SPI 提交失败沿用驱动下一周期重试。CPU 死机、定时器任务停顿、LED / SPI6 故障时，
灯可能停留在最后颜色或熄灭，不能把绿灯当作硬件看门狗，也不能靠此灯检测整个 CPU 死机。

## 验证记录

已完成：生产诊断代码的主机回归，覆盖全部 14 个故障位、颜色 / 次数、优先级与恢复、
150 / 300 / 900 ms 节拍边界、蓝灯 500 ms 边界、启动宽限、主动失能不误报、
卡弹锁存、1 s 使能观察超时、50 ms 快照时效边界、初始化失败和 indicators 关闭。
GimbalBoard、SingleBoard、ChassisBoard 固件构建及差异检查通过。
主机测试仅替换应用诊断输入、系统时间和 LED 提交，不替代真实 SPI / CAN 通信。
临时回归工程在 `/tmp/h7-ws2812-tests`，编译仓库当前诊断源码，执行：

```sh
cmake -S /tmp/h7-ws2812-tests -B /tmp/h7-ws2812-tests/build
cmake --build /tmp/h7-ws2812-tests/build
ctest --test-dir /tmp/h7-ws2812-tests/build --output-on-failure
```

临时目录不属于固件仓库，清理后需恢复回归工程；固件分支不引入 Tests/。

待实机验证：

- [ ] 烧录后复位，确认蓝色启动等待与绿色正常状态。
- [ ] 逐项中断遥控转发、DM-IMU、Pitch、左右摩擦轮、拨弹盘反馈，核对对应灯色与次数。
- [ ] 同时产生多项异常，确认优先级、位图及修复后的下一故障显示。
- [ ] 核对实际 WS2812 颜色、15% 亮度以及 SPI6 发送情况。
- [ ] 验证卡弹回退超时青色两闪、OFF 复位与旧动作不补射。
- [ ] 停止 ControlTask 而保留 TIM_1ms_Task，确认白色三闪。

实机项目尚未执行，以上勾选项不能作为已通过的测试。
