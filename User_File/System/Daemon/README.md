# 周期数据源在线监控

正常工作时应持续收到反馈、心跳或数据流的模块，优先持有静态 `Daemon`，在显式初始化时注册到 `DaemonManager`，只在协议和数据校验通过后 `Feed()`。使用固定容量 32 的指针注册表，无 malloc/new、虚函数或运行期注销。重复注册同一对象幂等；注册失败必须让模块初始化失败或保持明确不可用。

`Daemon` 只回答数据源是否活跃、离线多久和发生了什么跃迁，不负责停机、清错、重启设备、消息路由或业务降级。Application 通过 Device 的可用性和业务数据新鲜度决定控制策略，不持有 Daemon 指针。

职责分层固定为：**Daemon = 周期数据源唯一的 liveness / Online / Offline 来源；Device = 对外暴露 online 与 fault / ready 等只读状态；Application/Diagnostics = 把这些状态映射为故障码与灯效；WS2812 = 只负责显示故障码。** Diagnostics 通过 Gimbal/Shoot 诊断快照和 Input/Device 状态读取 Daemon 结论，不直接持有或遍历 DaemonManager，也不自行实现 `last_rx + timeout` 的设备掉线判断。控制 freshness（`Topic::ReadFresh`、timestamp age）回答"这份数据是否可用于当前控制周期"，与 Online 相互独立：Online 时旧数据仍可因过期被拒绝，严格 freshness 也可以先于 Daemon 超时失效。

## 时间与跃迁

- 复用系统绝对时间戳，内部使用自然回绕的 uint32_t 毫秒计数。新对象初始 Offline，收到第一份合法数据后才 Online。
- `Feed()` 可从 ISR 调用；读写状态用短 PRIMASK 临界区保护并恢复原中断状态。
- `IsOnline()` 按查询时刻判断 `age < timeout`，不必等待 StatusTask。它不消费跃迁、不执行回调。
- StatusTask 每 10 ms（100 Hz）统一 `CheckAll()`，报告一次 OfflineToOnline 或 OnlineToOffline；稳定状态返回 None。离线回调仅在 Check 观察到 OnlineToOffline 时调用一次，在临界区外、StatusTask 上下文执行。默认不设置 callback。
- `OfflineDurationMs()` 从实际超时点计算持续时间，在线为 0；初始未收到数据时从系统零时刻计时。`LastTransition()` 用于观察最近记录的跃迁。
- `SetTimeoutMs()` 仅用于初始化、首次 Feed 前配置，拒绝 0；不能在运行期改变反馈门限。

监控和实时控制有意分离：Daemon 的毫秒门限服务于模块诊断；Topic `ReadFresh()` 和电机微秒反馈快照决定具体数据能否进入 1 kHz 控制。不要以 CheckAll 的缓存状态替代实时 freshness。

## 当前接入

| 数据源 | 何时 Feed | 监控超时 | 实时/业务判断 |
| --- | --- | --- | --- |
| DM | 总线、ID、DLC、节点校验通过的运动反馈；模式应答不 Feed | 100 ms | 原有 requested/actual/fault/ready 和微秒快照保持 |
| DJI | 总线、CAN ID、DLC、encoder 全部合法且反馈解码完成 | Init 的 feedback_timeout_ms，默认 20 ms | 微秒反馈快照；Control/Send 过期清零，不等待 CheckAll |
| DM-IMU | 欧拉角帧寄存器、长度合法且三轴解码完成；角速度帧与非法帧不 Feed | 100 ms | 姿态数据另有 100 ms freshness（INS 桥 / GetPitch），Gimbal 按 INS Topic ReadFresh 失能 |
| 0x065 板间遥控转发 | 总线、ID 匹配、长度至少 6 byte 且三通道解码完成 | 100 ms | 通道快照另有 100 ms 接收时间戳 freshness；RemoteInput 失效提交空输入 |
| S.BUS | 完整合法 header/footer 的 25 字节帧，含 frame-lost/failsafe 帧 | 100 ms | Healthy 另看失控位；RemoteInput 50 ms freshness、200 ms 回中恢复保持 |
| INS | BMI088 已初始化且六个姿态/角速度值 finite，实际发布 INS_State 前 | 30 ms | Gimbal 10 ms ReadFresh 保持；原始 SPI chunk 不 Feed |
| BoardTransport | 总线/ID/大小正确、按 RX 时间仍及时、Decode 成功，序号过滤之前 | 100 ms | 重复序号维持链路在线但不刷新 Topic，业务 Topic 可独立过期 |
| Referee | CRC8/CRC16 和完整长度校验通过，已知命令长度正确；未知合法命令只证明链路活性 | 500 ms | Online 不保证每个业务字段已更新 |
| VTM | 完整 CRC16 合法遥控帧，或 CRC8/CRC16 合法图传链路帧 | 300 ms 暂定 | 需按实机帧周期验证；Online 不代表已实现所有业务载荷 |

Referee/VTM 的 C 解析器通过模块内的薄 C 接口访问静态 C++ Daemon，保持原有 C 编译和解析边界。它们当前未在 System_Init 自动绑定 UART；调用各自 Init 后才注册和 Feed。

未来正式接入 Vision 时，应在合法视觉帧后 Feed 独立 Daemon；瞄准目标仍按输入时间戳判断 freshness。当前没有正式 Vision 驱动，不新建占位实现。历史 DBUS 与 QDrive 当前不在机器人周期控制链绑定，本轮未扩展这些旧入口；正式接入时须按同一原则迁移。Buzzer、WS2812、Key、Flash、单次命令接口不因存在对象就注册 Daemon。

## 容量核算

按现有静态应用实例，SingleBoard 取三个硬件应用全部开启，并计入可选 Referee/VTM 初始化：

| 板型 | DM | DJI | S.BUS | INS | Transport | Referee | VTM | 合计 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| SingleBoard | 2 | 11 | 1 | 1 | 0 | 1 | 1 | 17 |
| GimbalBoard（框架） | 2 | 3 | 1 | 1 | 1 | 0 | 0 | 8 |
| ChassisBoard | 0 | 8 | 0 | 0 | 1 | 1 | 1 | 11 |

上表按框架默认装配统计；SingleBoard 默认关闭三个硬件应用，且不编入 DM-IMU/0x065 源，Referee/VTM 默认未初始化，实际注册数通常更少。GimbalBoard 不编入 Referee/VTM，ChassisBoard 不配置 INS/S.BUS。老步兵云台板（LEGACY_INFANTRY_GIMBAL）实际接入为 3 DM（Pitch + 左右摩擦轮）+ 1 DJI（拨弹盘）+ DM-IMU + 0x065，合计 6：该配置以 DM-IMU 替代 BMI088 INS、以 0x065 替代 S.BUS，且不初始化框架 Transport。

CAN 接收注册器目前总容量为 16，DM/DJI/Transport 共享这个上限；加上当前其他四个独立软件数据源，扩展电机数量时的成功注册上界仍最多 20。DaemonManager 保持 MAX_DAEMONS=32；未来增加模块时重新核算，并对注册失败作显式处理。
