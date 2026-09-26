# PowerControl 主机验证

本测试编译生产功率控制、RLS 与 Basic 源码，验证预算限幅、制动时不抵扣回馈功率、低于基线功率时的输出、RLS 收敛，以及缺失或无效输入时清零输出。

在本目录执行 `cmake -S . -B build`、`cmake --build build`、`ctest --test-dir build --output-on-failure`。

功率模块接收电机输出轴力矩（N·m）、同轴转速（rad/s）和本周期预算（W），返回限幅后力矩。`Update_Identification` 只接收新的一帧、与实际电机力矩及速度时间对齐的功率测量；重复旧帧或失效反馈不得调用。模块不读取裁判系统、不直接发送电机命令，也不声称主机测试等于整车功率验收。
