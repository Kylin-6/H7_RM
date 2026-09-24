# 初始化失败回归

在仓库根目录使用主机编译器运行：

```powershell
cmake -S Tests/Initialization -B build/Initialization -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Initialization
ctest --test-dir build/Initialization --output-on-failure -V
```

直接编译生产 BMI088（含总入口、加速度计、陀螺仪和 VQF）、W25Q64JV、System_Init
源码；不复制或改写被测函数。硬件桩在延时期间交付接收回调，以检查初始化阶段
仍能处理读回结果。系统启动测试单独注入设备 Init 成败，验证错误位和 FIFO 启动条件。

覆盖正常启动、无芯片 ID、复位后失联、非零/零值配置写入失败、第五次响应成功、
第六次响应判失败、丢失回调、错误软复位地址、BMI088 部分/全部失败后的运行入口、
Flash 第四字节残留、旧 ID 不得冒充新响应、内存映射失败，以及失败后的 Flash 操作拒绝。
每组还有进程超时保护。

另有 Flash 提交失败与 Quad 配置回归：注入 OSPI 的 ERROR/BUSY/TIMEOUT，验证读、写、
擦除和自动轮询拒绝时立即返回失败，不继续后续步骤；正常 Quad 流程检查 WIP 读取的
数据阶段。当前共 7 组测试，主机模型不模拟真实 Flash 写入耗时或总线电气条件。

资料依据：

- [Bosch 官方寄存器定义](https://github.com/boschsensortec/BMI08x_SensorAPI/blob/master/bmi08_defs.h)：ACC_PWR_CTRL 为 0x7D，ACC_SOFTRESET 为 0x7E；工作模式为 0，挂起模式为 3。
- [Bosch 官方复位实现](https://github.com/boschsensortec/BMI08x_SensorAPI/blob/master/bmi08a.c)：软复位后等待，并通过 SPI 芯片 ID 读取恢复通信。
- [达妙参数应答与模式切换示例](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)：0x55 为写参数，RID 10 为控制模式；相应回归位于 Tests/Boundary 的 commands 组。

5 次重试是本工程的有界启动策略，并非芯片手册规定。测试验证软件控制流、
寄存器地址和协议解析；真实 SPI/DMA 时序、器件电气条件和传感器输出仍须板上验证。
