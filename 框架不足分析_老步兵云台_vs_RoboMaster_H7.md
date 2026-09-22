# 框架不足分析：老步兵云台分支 vs RoboMaster_H7 分支

> 对比对象：`老步兵云台`（b727d51）vs `RoboMaster_H7`
> 性质：只读分析，未改动任何代码。
> 背景：老步兵云台分支比 RoboMaster_H7 多 5 个提交，把老云台板（DM Pitch + DM 摩擦轮 + M2006 拨弹盘，输入来自底盘板 0x065 板间帧）移植到 H7_BSP 框架上。**移植过程中被迫改动的框架文件，就是框架不足的直接证据。**

## 一、移植时被迫侵入修改的框架文件（核心证据）

| 文件 | 被迫改动 | 暴露的问题 |
|---|---|---|
| `Core/Src/freertos.c` | 用 `#if LEGACY_INFANTRY_GIMBAL` 守卫 BMI088Task 创建 | 任务创建表在 CubeMX 生成文件里，框架不拥有任务生命周期 |
| `System/Init/Init.cpp` | 60+ 行 `#if`：不绑 SPI2 回调、跳过 W25Q64、不初始化 ADC、电源配置不同 | System_Init 是单板硬编码，没有板级配置（Board Profile）层 |
| `Task/TIM_1ms_Task.cpp` | PulseEntry 静态表逐条 `#if` | 1ms 周期回调表不支持条件注册/运行时注册 |
| `System/callback/callback.cpp` | EXTI 回调需 `#if` 守卫，否则解引用未初始化的 `BSP_BMI088` | 中断分发直接引用全局单例，无"未注册即忽略"保护 |
| `Task/Control_Task.cpp` | 插入 `Communication_Init/Update` 并规定其在 RobotCmd 之前 | 应用模块调度顺序硬编码在调度器里 |
| `Task/TransportTask.cpp` | 整段 JustFloat 遥测通道布局写进任务 | 遥测无注册机制，每块板都要改传输任务源码 |
| `CMakeLists.txt`（根） | 新 option + 互斥 FATAL_ERROR + 条件源文件 + `OR` 宏拼接 | 板型 = 手写 CMake 逻辑，无 per-board 配置文件 |

**结论：换一块板 = 改 7 处框架/系统文件。理想状态应该是：换板只新增"板级描述 + 设备实例清单"，框架文件零改动。**

## 二、框架自身缺陷清单

### 1. 缺少板级配置层（最大短板）
`System_Init` 把"这块板有什么、先初始化什么"写死在源码里。需要的是类似这样的声明式机制：
- 每板一个 `board_xxx.cmake` / 板级配置头，声明：启用哪些任务、绑定哪些外设回调、哪些设备存在；
- `System_Init` / freertos 任务创建 / Pulse 表 / 遥测表全部由配置驱动生成或注册。

### 2. 初始化无失败隔离
`Class_W25Q64JV::Init()` 对 JEDEC ID `while` 死等（100ms 一次轮询、无超时无退出）。缺片/坏片会**卡死整个 System_Init**。移植时只能在板级层"跳过 W25Q64"绕过，而不是修框架。与运行期已有的 `Daemon` 在线检测体系不对称：**运行期有守护，初始化期没有**。建议：所有外设 Init 带超时与失败返回，失败设备标记 degraded 并继续启动。

### 3. 中断回调依赖全局单例且无兜底
`HAL_GPIO_EXTI_Callback` 直接调 `BSP_BMI088.EXTI_Flag_Callback()`，设备不存在就只能靠编译开关裁掉。若回调表改为"注册制 + 空槽忽略"（CAN 侧的 `BSP_CAN_RegisterCallback` 已经是这个正确方向），EXTI/SPI/TIM 回调也应统一。

### 4. 周期回调表静态写死
`TIM_1ms_Callback_Table[]` 需要 `#if` 裁剪条目。Pulse 表应支持各模块自行注册（带条件），而不是调度文件感知所有模块。

### 5. 数据流三套并存，消息中心用得不彻底
- Gimbal ↔ 消息中心：`Publisher/Subscriber`（正确姿势）；
- Com → RobotCmd：全局函数 `RobotCmd_SetGimbal()`；
- Pitch → DM_IMU：直接函数调用 `DM_IMU_GetPitch()`；
- TransportTask 读遥测：专用读出函数 `Communication_GetRawChannels()` / `Shoot_GetDebug()`。

同一框架里 Topic、全局 setter、直接调用、专用 debug 读出四种通路并存。**输入源/命令/反馈/遥测若都收敛到消息中心 Topic，新增输入适配模块就不需要改 Control_Task 的调度顺序。**

### 6. 设备接口风格不统一
DM_IMU 是 C 风格自由函数（`DM_IMU_Init/GetPitch`），与 `Class_*` 设备类、`sys_imu`（BMI088 系统级 IMU）三套风格并存。没有统一 IMU 抽象（如 `IImu::GetEuler/GetGyro/IsOnline`），应用层被具体 IMU 实现绑死。顺带：这次还发现 `Class_DMMotor` 缺 `IsEnabled()`（在线 ≠ 使能），是设备能力约定不全的一个实例。

### 7. 遥控输入源无抽象
RM 分支输入 = SBUS 遥控器；老云台板输入 = 底盘板 CAN 转发（0x065）。两者语义等价（火控/拨盘/通道值），但框架没有统一的"输入源"接口，导致要新建 Com 模块 + ChassisBoard 设备类 + 改调度顺序才能接入。理想是：`IRemoteSource`（SBUS / 板间链路 / 视觉）→ 统一产生 `OperatorCmd`。

### 8. Application 模块用编译开关内部分叉
`Gimbal.h` 内部 `#if LEGACY_INFANTRY_GIMBAL` 定义两个不同的 struct + extern 全局对象。同一模块按板型分叉数据结构，长期会越长越乱。更健康的做法是每板独立模块文件（`gimbal_qd.cpp` / `gimbal_legacy_pitch.cpp`），由构建配置二选一，模块对外接口不变。

### 9. 遥测机制缺失
JustFloat 通道布局、周期、缓冲全部手写在 TransportTask。需要：模块把可观测信号发布为遥测 Topic，传输任务统一收集；或提供遥测注册表（通道名 → 数据指针/回调）。`Debug_IMU_Data` 全局结构 + EricTool 绑定也是同类问题。

### 10. 双板协议无共享定义
0x065（遥控通道）/0x070/0x075 的编解码，云台板侧在 `chassis_board.*`，底盘板侧在另一分支的 `gimbal_board.*`，没有共享协议头。双板协议应放公共目录（或独立 protocol 仓库），避免两端漂移。

### 11. 任务层与设备层耦合
`user_task.h` 直接 include `bsp_bmi088/bsp_key/bsp_usb/dvc_erictool` 等设备头。任务文件本应只依赖调度与消息中心。

## 三、改进优先级建议（未实施）

1. **P0 — 板级配置层**：把 Init/任务创建/Pulse 表/遥测的 `#if` 收敛到每板一份配置，框架文件恢复零板型知识；
2. **P0 — Init 失败隔离**：外设 Init 统一超时 + 降级启动（先修 W25Q64 这类死等）；
3. **P1 — 回调注册制统一**：EXTI/SPI/TIM 对齐 CAN 的注册模式；
4. **P1 — 数据流收敛**：输入源抽象 + 遥测 Topic 化，Control_Task 不再感知具体模块顺序；
5. **P2 — 设备接口规范化**：IMU 统一抽象、设备能力约定（IsEnabled 等）；
6. **P2 — 双板协议共享头**。

## 四、值得肯定的方面

- CAN 回调注册制（`BSP_CAN_RegisterCallback`，按 (总线， ID) 注册）是正确方向，新设备（DM_IMU、ChassisBoard）零框架改动接入；
- 消息中心 Topic 的 PRIMASK 临界区 + trivially-copyable 约束设计合理；
- Daemon 在线检测、分层目录（Application/Device/Middleware/System/Task）清晰；
- 老分支本身的移植纪律很好：控制律逐项保持、差异全部注释说明、互斥配置有 FATAL_ERROR 兜底。
