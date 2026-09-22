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
- 构建树：build/gimbal（云台板 Yaw 关）、build/gimbal-yaw、build/rm-standard（标准 RM 板型）；host 测试 build/Tests_*（boundary_tests 需传用例名 pid/kalman/commands/parser）。

## 框架约定
- 控制律迁移纪律：替换手写算法前必须先做 host 数值等价对拍（参考 2026-09-22 的 slope/jam FSM 对拍模式，存 build/tmp/ 下可复用）。
- 设备接入在线检测用 Daemon + DaemonManager::Register；离线恢复用 Daemon 离线跃迁回调（dmmotor OfflineCallback 为范例）。
- 状态机用 Class_FSM（Count_Time 周期计数即 ms）；超时判定不再手写 tick 差值。
- 算法复用优先级：Class_Trajectory（S 曲线）/ Class_Slope（斜坡）/ Class_Filter_IIR_First_Order（一阶低通，注意变采样间隔场景不适用）/ Basic_Math_Constrain。
- FDCAN 三总线 AutoRetransmission 统一 DISABLE，使能帧靠软件兜底（Daemon 回调 + 应用层周期补发）。
- 排查文档：《重复造轮子排查_云台板移植代码.md》《框架修复清单.md》为活文档，修一项勾一项。
