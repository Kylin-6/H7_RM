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
- 新建构建树必须带 `-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`，否则 linker guard（h7_linker.cmake）FATAL_ERROR。
- 配置失败过的构建树缓存被污染，必须删除重建。
- 2026-09-23 起单一构建：CMakePresets 只留 Debug（build/Debug）；根 CMakeLists 已删除全部 option 多板型开关，宏硬编码为老步兵云台板（GIMBAL=1/CHASSIS=0/SHOOT=1/LEGACY_INFANTRY_GIMBAL=1/LEGACY_INFANTRY_GIMBAL_YAW=0），Pitch.cpp、dvc_dm_imu.cpp、chassis_board.cpp 无条件编入。原 build/gimbal、build/gimbal-yaw、build/rm-standard 多板型构建树已废弃。host 测试 build/Tests_*（boundary_tests 需传用例名 pid/kalman/commands/parser）。
- 双板分工（2026-09-23 用户确认）：Yaw 轴由底盘板主控控制，云台板固件固定不编译 Yaw（LEGACY_INFANTRY_GIMBAL_YAW=0 是架构决定，不是临时措施）；LEGACY_INFANTRY_GIMBAL_YAW 相关的 Yaw 代码块保留在源码里但永不启用。

## 框架约定
- 控制律迁移纪律：替换手写算法前必须先做 host 数值等价对拍（参考 2026-09-22 的 slope/jam FSM 对拍模式，存 build/tmp/ 下可复用）。
- 设备接入在线检测用 Daemon + DaemonManager::Register；离线恢复用 Daemon 离线跃迁回调（dmmotor OfflineCallback 为范例）。
- 状态机用 Class_FSM（Count_Time 周期计数即 ms）；超时判定不再手写 tick 差值。
- 算法复用优先级：Class_Trajectory（S 曲线）/ Class_Slope（斜坡）/ Class_Filter_IIR_First_Order（一阶低通，注意变采样间隔场景不适用）/ Basic_Math_Constrain。
- 老步兵云台 FDCAN1 AutoRetransmission 保持 ENABLE（fdcan.c 与 .ioc 同步）；FDCAN2/3 保持 DISABLE。关闭 FDCAN1 后用户反馈右摩擦轮持续不转，已恢复配置、待上板复验；Daemon 回调与周期使能补发无法兜底在线且已使能时的速度帧仲裁丢失，不能据此关闭硬件重传。
- 排查文档：《重复造轮子排查_云台板移植代码.md》《框架修复清单.md》为活文档，修一项勾一项。
