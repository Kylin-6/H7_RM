# 统一在线监控主机回归

本目录只保存在测试分支；测试编译同步后的生产源码，不将 Tests 合并回 RoboMaster_H7。

```sh
cmake -S Tests/Daemon -B build/host-daemon -G Ninja
cmake --build build/host-daemon
ctest --test-dir build/host-daemon --output-on-failure
```

九组独立进程验证：Daemon 初始离线、恢复、超时、单次跃迁与回调、时间回绕、32 项注册容量及幂等；DJI 合法反馈、非法 bus/ID/DLC/encoder、即时 freshness 和清零；S.BUS 合法性、失控帧活性及 RemoteInput 50 ms/200 ms 行为；INS finite 发布与 30 ms 诊断/10 ms Topic freshness 分离；Referee CRC 与 500 ms 超时；VTM CRC、长度与 300 ms 超时；各模块注册失败保持不可用。

Referee/VTM 按 C 编译，通过生产 C 桥连接 C++ Daemon。硬件总线、时钟和 BMI088 输出使用替身，协议、Daemon、Topic、输入仲裁和 DJI 控制路径使用生产实现。Transport 的重复序号及 Topic 过期检查位于 Tests/Transport；Gimbal、Chassis 和启动回归位于各自目录。主机测试不验证实机中断时序、UART DMA 或 CAN 总线。
