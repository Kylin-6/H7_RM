# 设备健康检查

`Status_Task` 每 50 ms 读取 IMU 和已注册 DJI 电机，更新 `SYS_Health`。只报告设备状态、异常原因和数据年龄；设备保护仍由原驱动执行，健康检查不发送控制指令、不清积分，也不修改电机在线标记。

## 任务与接口

`H7_BSP.ioc` 中的 `StatusTask` 使用 CMSIS-RTOS V2、Low 优先级、动态分配和 512 words（2048 字节）栈。`freertos.c` 的任务属性、创建入口和弱函数与 IOC 同步，具体实现位于 `User_File/Task/StatusTask.cpp`。

任务启动时调用 `Sys_Health_Init()`，随后周期调用 `Sys_Health_Update()`。使用 `osDelayUntil`，周期按系统 tick 向上取整；更新超时或唤醒过晚时跳过错过的周期，不连续补跑。

参数集中在 `sys_health.h`：检查周期 50 ms、IMU 超时 100 ms、首帧宽限 1000 ms、恢复需要连续两次正常。电机沿用各自初始化配置中的反馈超时。

## 状态

| 状态 | 含义 |
| --- | --- |
| `HEALTH_DISABLED` | 尚未检查或未注册 |
| `HEALTH_STARTING` | 启动宽限内等待首份数据 |
| `HEALTH_OK` | 数据新鲜且没有新增异常 |
| `HEALTH_WARN` | IMU 异常计数发生变化 |
| `HEALTH_OFFLINE` | 等待首帧或数据年龄超过超时 |
| `HEALTH_FAULT` | 数据时间戳超前于系统时间 |

超时采用严格大于比较。每个设备从首次被检查开始计算首帧宽限，运行中后注册的电机也有自己的宽限。异常后第一次正常仍保留原异常状态，原因变为 `HEALTH_REASON_RECOVERING`；第二次连续正常才进入 `HEALTH_OK`。

IMU 复用原调试数据中的九项累计计数，检查 SPI 错误/超时、恢复、丢样、拒绝更新、时间戳异常和 VQF 复位。首次有效数据只建立基线；之后只对新增变化告警，计数回绕同样视为变化。数据年龄取实际样本与调试快照中较旧的时间戳，因此实际采样停止仍可被识别。原 `Debug_IMU_Data` ABI v5 / 264 字节保持不变。

DJI 复用原注册表，最多 24 个；`Motor[i]` 按注册顺序排列，通过 `Bus` 和 `Feedback_Id` 对应设备。`Enabled=false` 只表示控制使能关闭。实例及 CAN 句柄的生命周期沿用原驱动要求。

## 读取状态

调试器查看 `SYS_Health.IMU` 和 `SYS_Health.Motor[i].Health` 的 `State`、`Reason`、`Data_Age_ms`。`UINT32_MAX` 表示无有效数据或年龄超出字段范围；通过 `Updated_At_us` 和 `Update_Count` 确认健康任务仍在运行。哪些设备异常影响整机，由应用层决定。

其他任务调用 `Sys_Health_GetSnapshot(&snapshot)` 获取一致副本。初始化和更新只由健康任务执行；读取接口在任务上下文使用，复制期间短暂屏蔽中断并恢复原 PRIMASK，不作为高频控制数据通路。

不停核 SWD 读取时，先读 `Sequence`，复制完整结构，再读一次 `Sequence`；前后序号和副本的 `Sequence/Sequence_End` 必须相同且为偶数。检查 `ABI_Version=2`、`ABI_Size` 和 `Update_Count>0`；结构布局以目标 ELF 为准。

## 验证

```powershell
cmake --build build/Tests_Health
ctest --test-dir build/Tests_Health --output-on-failure
cmake --preset Debug
cmake --build --preset Debug
```

首次配置主机测试时运行 `cmake -S Tests/Health -B build/Tests_Health -G Ninja -DCMAKE_CXX_COMPILER=g++`。

五组回归覆盖快照与 IRQ 状态、IMU 首帧/超时/九项增量异常及恢复、DJI 只读超时与后注册首帧宽限、24 电机容量、不同 tick 频率下的周期取整/回绕/超时跳过。测试编译生产健康模块、任务及 DJI 驱动，用桩替代硬件和 RTOS 调度。

2026-09-21：五组主机回归及 Debug、Release 构建通过。Release 使用 DTCMRAM 106488 B、FLASH 102208 B。相对精简前的 Debug 固件：

| 项目 | 精简前 | 精简后 |
| --- | ---: | ---: |
| DTCMRAM 使用量 | 108664 B | 106496 B |
| FLASH 使用量 | 119452 B | 118696 B |
| 健康状态机/恢复计数 | 1300 B | 25 B |
| IMU 历史数据 | 264 B | 36 B |
| 单份健康快照 | 928 B | 616 B |

任务栈仍从已预留的 RTOS heap 中分配。IOC 与对应任务代码已同步，本次没有运行 CubeMX 全工程再生成、烧录或板上时序测量；任务栈水位和对控制周期的影响仍需实测。
