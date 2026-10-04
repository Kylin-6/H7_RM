# 老步兵底盘 WS2812 指南

适用于 ChassisBoard 构建（Diagnostics 模块编入）。参考老步兵云台状态灯：单颗 SPI6 WS2812，亮度 15%；只观察，不改变控制许可、设备输出或故障恢复。

## 在线状态来源与 freshness

各周期数据源的 Online / Offline（liveness）唯一由 Device 内的 Daemon 判定
（详见 `User_File/System/Daemon/README.md`）：四轮与 Yaw 为各自 DMMotor Daemon，
遥控为 SBUS Daemon，INS 为 System_IMU Daemon。Diagnostics 只消费这些
Device / Application 诊断快照并映射为故障码（遥控位经
`RemoteInput_IsLinkOnline()` 读取 SBUS Daemon 结论），不自行计算
`last_rx + timeout`，WS2812 只负责显示。

与 Online 相互独立保留的数据 freshness（"这份数据是否可用于当前控制"）：
INS Topic `ReadFresh`（控制侧 10 ms 门限）、S.BUS 50 ms 帧新鲜度与失控位
互锁、电机微秒反馈时间戳和使能观察超时。因此可能出现"Daemon 仍在线但旧
姿态已被控制拒绝"，或"严格 freshness 先于 Daemon 超时失效"。

## 看灯判断

故障每次亮 150 ms、灭 150 ms，一组结束后额外灭 900 ms，再重复。
轮编号 1～4 对应 `motor_id[]` 顺序，即节点 0x01～0x04、反馈 0x60～0x63，不能仅凭编号推断实车安装位置。

| 灯色 | 闪烁次数与含义 |
| --- | --- |
| 白 | 1：系统致命初始化失败；2：底盘、遥控或命令入口初始化失败；3：控制诊断超过 50 ms 未更新或从未发布 |
| 黄 | 1：S.BUS 遥控链路离线（SBUS Daemon）；2～5：第 1～4 路轮电机请求使能连续 1 s 未就绪 |
| 品红 | 1：BMI088 INS 姿态无新鲜数据（沿用控制侧 10 ms 门限；链路活性见 INS Daemon） |
| 红 | 1～4：对应轮电机最近反馈 state > 1 |
| 橙 | 1～4：对应轮电机请求工作但反馈离线（DMMotor Daemon） |
| 青 | 1：Yaw 请求工作但离线（DMMotor Daemon）；2：Yaw 最近反馈 state > 1；3：Yaw 请求使能连续 1 s 未就绪 |
| 绿常亮 | 有控制许可，INS 与所需电机正常；不表示轮子一定在转 |
| 蓝慢闪 | 启动或使能等待，亮、灭各 500 ms |
| 蓝常亮 | 主动撤销许可，且无其他显示异常 |

优先级：系统初始化 → 应用初始化 → 控制快照过期 → 遥控 → INS → 四轮硬件故障 → 四轮使能超时 → 四轮离线 → Yaw 硬件故障 → Yaw 使能超时 → Yaw 离线。同类轮故障先显示编号较小的一路；修复后自动显示下一项。

开机前 3 s 只显示初始化失败，其他状态先蓝慢闪。主动失能不因电机停止反馈而报警；最近的电机故障状态即使反馈过期仍保留，收到非故障反馈后清除。使能超时只是观察，不影响现有 ServiceAll 重试。遥控关闭仍显示黄色链路异常，而非蓝色主动撤销。

## 诊断与刷新

ControlTask 在 Chassis_Update 后每 10 ms 调用 `Diagnostics_Publish()`，发布 `MessageCenter::Chassis_Diagnostic_Topic`，含 `fault_mask / permitted / waiting`。TIM_1ms_Task 的既有 10 ms 回调调用 `Diagnostics_LED_Update()`，读取一致快照、选择灯效并刷新 SPI6，输入模块不再写灯色。`BoardConfig.indicators=false` 时不设置和发送灯效。

故障位图：bit 1 应用初始化，bit 3 遥控，bit 4 INS，bit 5～8 四轮故障，bit 9～12 四轮离线，bit 13～16 四轮使能超时，bit 17 Yaw 故障，bit 18 Yaw 离线，bit 19 Yaw 使能超时。系统致命失败 bit 0 与控制快照过期 bit 2 由灯效读取时叠加。低优先级异常保留在快照，不因未显示而清除。

灯效保持单任务颜色设置和发送；SPI 提交失败由原驱动下一次重试。CPU、定时器任务或 SPI6 停止时，灯可能保持最后颜色或熄灭，不能替代硬件看门狗。仅控制任务停止而定时器任务仍工作时，可显示白色三闪。

模块结构与老步兵云台一致：`Application/Diagnostics/Diagnostics.cpp` 负责诊断组合、优先级和节拍纯计算，`Diagnostics_Runtime.cpp` 负责采集、发布及灯色刷新；`Chassis_GetDiagnostic()` 仅在 ControlTask 提供设备状态。`Diagnostics_LED_State` 由定时器任务保留完整故障位图和当前灯效，供调试器观察。

## 验证范围

本次执行三板型构建、差异检查和灯效主机验证；主机验证替换时间、快照与 LED 输出，使用生产灯效函数，检查颜色、优先级、闪烁边界、启动宽限和 indicators 关闭。实机灯色、亮度、遥控/IMU/电机断线恢复和控制任务停止提示尚未烧录验证。
