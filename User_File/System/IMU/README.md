# INS 状态发布与在线监控

`System_IMU_Configure()` 设置整机 VQF 参数并注册静态 INS Daemon，必须在 BMI088 Init 前调用；注册失败返回 false，System_Init 记录 BMI088/INS 链路不可用并禁止启动采集。

`System_IMU_Publish_State()` 只在 BMI088 已初始化且 Yaw/Pitch/Roll、三轴机体系角速度全部 finite 时 Feed 并发布 INS_State。NaN/Inf 不发布，也不刷新在线时间；原始 SPI/DMA chunk 不 Feed。因此 `System_IMU_IsOnline()` 监控整个采集、解算到有效 INS 输出链路。

监控门限为 30 ms，用于输出链路诊断；BMI088Task 按 FIFO 队列批量解算并在每批完成后发布一次，发布频率不等于陀螺仪采样频率；Gimbal 仍以 `INS_State_Topic.ReadFresh(..., 10000)` 判断 10 ms 实时控制时效。二者不合并。StatusTask 100 Hz 统一 CheckAll，Daemon 不控制云台模式或整车安全策略；实现不使用动态内存。参见 [Daemon 说明](../Daemon/README.md)。

BMI088 加热已开启（`BMI088_Accel.Init(true)`）：TIM3 CH4 加热 PWM 与 128 ms 温控
PID 生效，目标温度 `HEATER_TARGET_TEMPERATURE = 50°C`；低于预热基点 45°C 时先以
固定预热功率加热，到达后切 PID 闭环，PWM 输出按当前电源电压对标称 25.2 V 做平方比
补偿。温度数据无效时加热比较值清零。VQF 姿态解算与零偏估计继续运行。
