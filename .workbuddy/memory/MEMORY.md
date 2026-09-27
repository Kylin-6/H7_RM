# H7_BSP 项目长期约定

## 提交规范
- 提交信息格式：`type: 中文描述`（英文半角冒号 + 空格）。type 取值：feat / fix / refactor / docs / merge。
- 禁止全角冒号（曾出现过 `docs：`，2026-09-22 决定：已推送的历史不改写，今后新提交必须半角）。
- merge 提交用 `merge: 合入 <来源>（<内容摘要>）`。

## 分支与工作区
- 主力分支：老步兵云台（云台板）、老步兵测试（底盘板）、RoboMaster_H7（框架主线）。
- 仓库存在多个会话并行操作：开工前必须 `git branch --show-current` 确认分支；跨分支对比配置必须在 checkout 之后再读文件。
- 领先远程的提交由用户决定何时 push，不主动强推。

## 构建（本机 Linux）
- /tmp 是 10MB tmpfs：编译必须 `env TMPDIR=$PWD/build/tmp cmake --build ...`。
- 配置失败过的构建树缓存被污染，必须删除重建。
- 2026-09-27 起回归框架多板装配构建（合并 RoboMaster_H7 时用户确认"全面对齐框架架构"，取代 2026-09-23 的单一 Debug 构建决定）：
  - 实机固件 = `cmake --preset GimbalBoard`（产物 build/GimbalBoard/GimbalBoard.elf）；
  - Debug/Release 预设 = SingleBoard 安全模板（H7_APP_* 默认 OFF），build/Debug/H7_BSP.elf；
  - 板型预设存 CMakeUserPresets.json；板级硬件（CAN 分配/imu/flash/adc/电源轨/indicators/usb_debug）在 User_Config/Board/*_board_config.cpp 的 BoardHardware 结构（含 power_24v_1/2 两路 24V 轨）；
  - 任务装配在 board_tasks_*.c（Board_CreateTasks），freertos.c 只留 USER CODE 区调用，CubeMX 再生成安全；
  - LEGACY_INFANTRY_GIMBAL=1 / LEGACY_INFANTRY_GIMBAL_YAW=0 仍全局硬编码，云台板专属源（Pitch.cpp、dvc_dm_imu.cpp、chassis_board.cpp、Com.cpp）挂在 GimbalBoard 装配段。
  - host 测试已迁到 RoboMaster_Test 分支，本分支不再有 Tests/。
- 双板分工（2026-09-23 用户确认）：Yaw 轴由底盘板主控控制，云台板固件固定不编译 Yaw（LEGACY_INFANTRY_GIMBAL_YAW=0 是架构决定，不是临时措施）；LEGACY_INFANTRY_GIMBAL_YAW 相关的 Yaw 代码块保留在源码里但永不启用。

## 命令链与板间通信（2026-09-27 合并后）
- 命令链回到框架 RobotCmd setter + 绑定 Output 模式：Communication → RobotCmd_SetGimbal/SetShoot → RobotCmd_Update 按变化发布；Output 未绑定直接拒绝启动。
- 框架 Transport 协议（System/Transport，0x141/0x222）已编译进 GimbalBoard 但 legacy 板不启用：老步兵板间链路仍是 0x065 遥控转发（Com.cpp + chassis_board），云台板不控底盘、无 ChassisCmd 下发。切 Transport 需底盘板（老步兵测试分支）同步适配。
- 框架 Gimbal 通用实现（配置驱动 DM 双轴）与 legacy 实现在 Gimbal.cpp/h 内 #if LEGACY_INFANTRY_GIMBAL 互斥共存；legacy 路径委托 Application/Pitch（DM MIT + DM-IMU），未上板复验。

## 框架约定
- 控制律迁移纪律：替换手写算法前必须先做 host 数值等价对拍（参考 2026-09-22 的 slope/jam FSM 对拍模式，存 build/tmp/ 下可复用）。
- 设备接入在线检测用 Daemon + DaemonManager::Register；离线恢复用 Daemon 离线跃迁回调（dmmotor OfflineCallback 为范例）。
- 状态机用 Class_FSM（Count_Time 周期计数即 ms）；超时判定不再手写 tick 差值。
- 算法复用优先级：Class_Trajectory（S 曲线）/ Class_Slope（斜坡）/ Class_Filter_IIR_First_Order（一阶低通，注意变采样间隔场景不适用）/ Basic_Math_Constrain。
- 老步兵云台 FDCAN1 AutoRetransmission 保持 ENABLE（fdcan.c 与 .ioc 同步）；FDCAN2/3 保持 DISABLE。关闭 FDCAN1 后用户反馈右摩擦轮持续不转，已恢复配置、待上板复验；Daemon 回调与周期使能补发无法兜底在线且已使能时的速度帧仲裁丢失，不能据此关闭硬件重传。
- 排查文档：《重复造轮子排查_云台板移植代码.md》《框架修复清单.md》为活文档，修一项勾一项。
