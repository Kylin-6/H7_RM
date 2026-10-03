# 统一在线监控主机回归

本目录只保存在测试分支；测试编译同步后的生产源码，不将 Tests 合并回 RoboMaster_H7。

```sh
cmake -S Tests/Daemon -B build/host-daemon -G Ninja
cmake --build build/host-daemon
ctest --test-dir build/host-daemon --output-on-failure
```

daemon_tests 十一组独立进程验证：Daemon 初始离线、恢复、超时、单次跃迁与回调、时间回绕、32 项注册容量及幂等；DJI 合法反馈、非法 bus/ID/DLC/encoder、即时 freshness 和清零；DM 合法反馈 Feed、非法总线/ID/长度/节点不 Feed、snapshot.online 与 Daemon 一致、100 ms 超时；DM-IMU 合法欧拉角 Feed、错误长度/角速度帧不 Feed、100 ms 超时、freshness 门限独立；S.BUS 合法性、失控帧活性及 RemoteInput 50 ms/200 ms 行为；INS finite 发布与 30 ms 诊断/10 ms Topic freshness 分离；Referee CRC 与 500 ms 超时；VTM CRC、长度与 300 ms 超时；各模块注册失败保持不可用。

legacy_input_tests 四组（LEGACY_INFANTRY_GIMBAL 路径）：0x065 ChassisBoard 合法 6 字节帧 Feed、长度不足不 Feed、Daemon 超时与通道 freshness 边界独立；RemoteInput 链路在线接口直接返回 ChassisBoard Daemon 结果，失效提交空输入；DM-IMU 桥 Daemon 在线但 INS Topic 对 10 ms 控制窗口过期必须被 ReadFresh 拒绝（Online 与 freshness 分离）；Diagnostics 故障位映射——设备 online=false 映射对应 offline 位、required=false 不误报、遥控/INS 输入独立。

Referee/VTM 按 C 编译，通过生产 C 桥连接 C++ Daemon。硬件总线、时钟和 BMI088 输出使用替身，协议、Daemon、Topic、输入仲裁和 DJI/DM 控制路径使用生产实现。Transport 的重复序号及 Topic 过期检查位于 Tests/Transport；启动回归位于 Tests/Initialization；主机测试不验证实机中断时序、UART DMA 或 CAN 总线。
