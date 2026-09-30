# 达妙云台回归

直接编译生产 Gimbal、Class_DMMotor、PID、Daemon 和 MessageCenter。替身只提供
CAN 注册/提交、寄存器中断屏蔽和统一时间源；反馈按实际 MIT 格式送入生产回调，
断言生产驱动编码后的 CAN 帧，不复制控制函数。生产依赖同步自 RoboMaster_H7
的 6bef0025 及本次 RequestEnabled 状态边沿修改。

```sh
cmake -S Tests/Gimbal -B build/Tests_Gimbal -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Tests_Gimbal
ctest --test-dir build/Tests_Gimbal --output-on-failure
```

共 20 组，其中 6 组验证 DMMotor：

- 初始 false 请求不发布、不提交，也不启动协议维护；false→true 只提交 Enable。
- READY 下连续 1000 次 true 请求不覆盖正常目标；true→false 先发布安全目标再提交 Disable；连续 1000 次 false 请求无动作。
- 安全发布与离散提交成功/失败的四种组合、边沿失败后相同请求无动作，以及在线反馈纠正失败的使能请求。
- 在线实际失能时纠正 Enable，在线实际使能时纠正 Disable；反馈一致时无命令；从未在线或反馈达到 100 ms 时不新增协议命令；故障时不自动使能、失能或清错。
- MIT/转矩、位置速度、速度、力位四模式在请求失能、实际失能、故障和反馈过期时的安全输出；ready 后可发布正常目标。反向配置的浮点负零按数值零验证。
- Snapshot、中断屏蔽恢复、发送失败返回和离线 Gimbal 场景。

另 14 组覆盖配置拒绝、初始化失败、部分初始化、READY/稳定 DISABLED 请求、Yaw 控制、Pitch 转换、方向翻转、模式切换、恢复重置 PID 与旧目标不重放、离线、发送失败、INS 新鲜度边界、默认参数和 100 Hz 反馈。
READY 无稳定窗口；离线不进行定时重试或进入超时 FSM；输入数值校验由 Application 输入边界承担，测试不把已删除的云台重复校验当作当前行为。

Tick 模拟 ControlTask，每次越过 10 ms 边界后调用一次 ServiceAll；直接 DMMotor 测试单独调用 ServiceAll。新增边沿回归在旧 RequestEnabled 实现下失败，更新后通过。

测试不模拟物理总线、硬件 FIFO 延迟、电机内部控制器或机构动力学；不能据此认定
示例 PID、MIT 增益、协议量程与机械限位已经适配某一实机。
