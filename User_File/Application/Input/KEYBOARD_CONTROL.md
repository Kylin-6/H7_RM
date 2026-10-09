# 老步兵键鼠控制

配套固件为「老步兵云台」的 GimbalBoard（Pitch、发射、UART7 接收）与「老步兵测试」的 ChassisBoard（四轮、Yaw）。SingleBoard 保持原模板。本次输入适配未修改闭环 PID、力矩、加速度、jerk 或 2 s Pitch 使能延迟，不能据此判断云台闭环滞后已解决。

## 知识库键位

| 输入 | 行为 |
| --- | --- |
| W / S | 前进 / 后退；同时按下抵消 |
| A / D | 左移 / 右移；同时按下抵消 |
| Q / E 按住 | 正 / 反向小陀螺；同时按下停止自转且不跟随 |
| Q / E 都松开 | 恢复普通跟随（相对 Yaw 反馈有效时） |
| Shift 按住 | 加速，松开恢复普通速度 |
| 鼠标 X / Y | Yaw / Pitch |
| 鼠标左键 | 释放短按单发；长按连发，释放停止连发 |
| Ctrl | 单独按下不切换跟随 |

Ctrl+Shift 组合优先：组合中的 WASD/Q/E 不产生平移、自转或跟随修正，鼠标仍可控制云台。B 爆发、V 能量机关、G 掉头、右键自瞄与整车组合快捷键仅保留原键位，不执行功能，也不映射为其他动作。完整 16 位键盘位图仍被转发。

平移由云台第一人称坐标旋转到底盘坐标，使用当前 Yaw 正前方标定与跟随符号。相对 Yaw 反馈不可用时无法保证方向，平移保持零，等待反馈；自转仍按 Q/E 意图生成。旋转与单轮目标继续经过现有规划和限幅。

## 参数归属和量纲

| 目标 | 参数与换算 |
| --- | --- |
| Pitch | `Gimbal_Config.h`：原始鼠标限幅 ±500 后整数 /5 截断，±100 映射 ∓`pitch_torque.target_rate_rad_s`（当前 ∓3 rad/s）；按实际 ms 积分，单次 dt ≤10 ms；角度夹到当前 `pitch_min/max`（−40°～+15°） |
| Yaw | 底盘 `Chassis_Config.h`：原始鼠标限幅 ±1000，一次缩放至 ∓`yaw_speed_max_rad_s`（当前 ∓8 rad/s），交给原姿态闭环和速度规划 |
| 平移 | 当前轴上限 ×普通比例 0.6 / Shift 比例 1.0，当前抽象目标 18 / 30；转坐标后仍受轴限幅 |
| 自转 | 当前旋转上限 ×0.3，Shift 再乘知识库倍率 1.5，当前 ±15 / ±22.5 |
| 发射 | `Shoot_Config.h`：键鼠长按 150 ms；小于 150 ms 的释放产生 ShootOnce，达到阈值提交 BURST，释放提交 STOP；原遥控开关仍用 300 ms |
| 摩擦轮 | 复用 Shoot 的 −25 / +25 rad/s、达速检查、热量、卡弹和 300 ms 延时停轮 |
| 拨弹盘 | 4500 rpm 转子速度 ×2π/60/减速比 36 = 13.08997 rad/s 输出轴速度；遥控波轮也引用这份换算 |

底盘抽象速度没有实测 m/s 标定，仍复用现有 `*_ToSi/FromSi` 在框架仲裁边界归一化，不把 18/30 解释成 m/s。参数集中于对应 Application 配置，输入层不重新标定电机。

## 接收、来源和恢复

云台 UART7 为 921600、8N1，PE7 为 RX、PE8 为 TX，与老工程引脚相同，IOC 与初始化同步。用户确认使用 VT02/VT12 常规版及 2025 国赛 Client；按官方串口协议 V1.9，图传链路为 921600，裁判系统常规链路为 115200。旧版 V1.7 的图传波特率为 115200，不能跨版本套用。参见 [官方串口协议下载](https://www.robomaster.com/en-US/resource/pages/announcement/1768)。

GimbalBoard 使用独立的 `vtm_legacy.c/.h`，保留 `vtm_26.c/.h` 给新图传 VT03/VT13 的 A9 53 协议。Legacy 的静态 255 字节缓冲跨 UART DMA chunk 拼帧，复用官方 CRC8/CRC16；CRC 失败逐字节重同步，拒绝超长声明，不动态分配或增加任务。`VTM_Legacy_ReadKeyboardSnapshot` 在恢复 PRIMASK 的短临界区复制完整快照。

VT02/VT12 键鼠使用 A5 帧：5 字节帧头（小端载荷长度、序号、CRC8）、2 字节小端命令 0x0304、12 字节载荷、2 字节 CRC16，总计 21 字节。载荷 byte0～5 为小端 int16 鼠标 X/Y/Z，byte6/7 为左右键（0/1），byte8～9 为小端完整 uint16 键盘，byte10～11 保留。键盘位序 W/S/A/D/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B 为 bit0～15。官方发送频率为 30 Hz。仅合法 0x0304 更新键鼠实际接收 ms、本地递增序号及帧头序号；其他合法图传帧只更新链路诊断，不续期键鼠。

模式按本次确认的 SBUS CH5：中心化值 <0 选择键鼠，其余档位选择遥控。CH5<0 不再是整车停机开关；CH7 速度、CH8 跟随保持遥控档原规则。SBUS 帧须年龄 ≤50 ms、无 frame_lost/failsafe，切源或故障后须连续健康 200 ms 才重新许可。键鼠档尚未收到合法图传数据或年龄 >50 ms 即停机，不自动退回遥控；新帧恢复后仍须满足 SBUS 与 INS 条件。VT02/VT12 没有 pause 或遥控挡位字段，不解析新图传的这些字段。

`InputState_SetPermission(permitted, require_remote)` 只由 ControlTask 调用；默认 `(true,true)` 保留原 Remote 互锁。键鼠使用 SBUS 所选来源的健康许可和 UART7 键鼠时效，云台同时要求新鲜有限 INS；遥控继续要求底盘下发遥控模式、许可和 50 ms 通道时效。底盘核对本地 CH5 与 0x066 模式一致，避免切档后执行旧模式。

云台顺序为 INS 桥 → KeyboardInput（模式、许可）→ RemoteInput → RobotCmd → Gimbal/Shoot；底盘为 INS 接收 → RemoteInput → KeyboardInput → RobotCmd → Chassis。设备/协议层只解析与缓存，Input 生成意图，RobotCmd 仍是唯一命令发布者。

进入键鼠、失联、切源或 Pitch 未就绪时不积累鼠标目标；从新鲜 INS 捕获当前 Pitch，在现有使能等待期间持续保持当前姿态，就绪后才积分。发射动作检测重置后先观察到左键或火控开关释放才接受新动作。无效 Remote/Keyboard 输入保留事件序号基线，防止恢复补发旧单发；Shoot OFF 沿用现有停轮和清事件路径。

## 双板协议

两板必须使用配套协议固件；不接受旧版无许可标记的 0x065。均为 FDCAN2、标准 ID、8 字节 Classic CAN，沿用 BSP 最新值槽；提交成功只表示软件接收目标，不确认对端执行。

### 0x066：云台 → 底盘

每 10 ms 刷新，模式或许可变化立即提交；提交失败在后续控制周期重试。

| 字节 | 内容 |
| --- | --- |
| 0 | 高四位版本 1，低四位模式 0/1/2 |
| 1 | uint8 递增序号 |
| 2～3 | 大端原始鼠标 X int16，限幅 ±1000 |
| 4～5 | 大端完整键盘 uint16 |
| 6 | bit0 运行许可，其余位 0 |
| 7 | 保留 0 |

停机或无许可时鼠标、键盘均清零。底盘检查总线、ID、精确长度、版本、挡位、保留位、数值范围及安全帧清零规则。首次收到前仍采用原遥控；收到后依帧模式运行，>50 ms 无新有效序号停机，不自动回退。

### 0x065：底盘 → 云台

原周期 2 ms 保持。byte0～1 火控、byte2～3 波轮、byte4～5 Pitch，均为大端 int16；byte6 高四位版本 1、bit0 为所选来源的健康许可、bit1～2 为模式 0/1/2、bit3 为 0；byte7 为递增 uint8 序号。只有 SBUS 健康并通过 200 ms 恢复条件才授予许可；仅已许可的遥控档携带三个通道，键鼠档、停机或未解锁时通道全零。云台用模式区分键鼠选择和零摇杆，用实际接收时间仲裁，不在 1 kHz 轮询时刷新时间戳。

两种协议的序号均只接受向前差值 1～127，255→0 合法，重复或回退不刷新业务快照。合法重复帧只更新链路流量基准；合法流量中断 >50 ms 后允许重建序号基准，以支持对端重启。非法帧不更新基准。协议没有会话号，间隔后的合法旧帧与重启首帧不可区分；当前不提供会话防重放。0x070/0x075 和 INS 回传 0x143/0x144 保持现有协议。

## 验证

在两份工作区分别运行：

```sh
cmake --preset SingleBoard
cmake --build --preset SingleBoard
cmake --preset GimbalBoard
cmake --build --preset GimbalBoard
cmake --preset ChassisBoard
cmake --build --preset ChassisBoard
git diff --check
```

主机检查在 /tmp 临时目录执行，不保留 Tests/KeyboardControl。已通过 VT02/VT12 拆帧、粘包、CRC、字段与快照时效，键位/参数、149/150 ms 发射、序号回绕、CH5 切源、SBUS/CAN 时效、恢复边界与云台释放门检查。两分支的 SingleBoard/GimbalBoard/ChassisBoard 构建通过，主要目标云台 DTCMRAM 88.57%、底盘 83.61%；固件构建仍有既有 PID linkage 等警告。

实机尚未验收：先确认方向、Pitch 限位及使能等待，再依次确认普通跟随、Q/E 自转、双键停止、松开恢复、Shift 加速和发射；测试 UART/CAN 拔线、CH5 切源、SBUS failsafe、单板重启与持住火控恢复。测量 1 kHz 周期耗时与链路刷新周期。尚未进行实机验收或烧录。
