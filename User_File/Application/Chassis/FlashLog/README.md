# 底盘 Flash 故障记录

ChassisBoard 使用底盘 OSPI2 上的 W25Q64（8 MiB）。ControlTask 在诊断发布后
每 10 ms 采样一次，只复制到 256 条静态 RAM 队列；低优先级 StorageTask 是 Flash
唯一读写者，使用已有 OSPI DMA 缓冲。队列在 `.ram_d1_data`，不是 DMA 的直接源。
写入不在控制任务或中断中执行，没有运行期动态分配。

控制任务从首次采样开始记录，不等待运动；停车、失能、IMU 过期也持续记录。
Flash 初始化和扫描期间先保留 RAM 样本，256 条可缓存约 2.56 秒；队列满时保留
已缓存的开机数据，丢弃后续样本并计数，Flash 就绪后排空队列。空白 8 MiB 可存 65536 条，即约
655 秒（10 分 55 秒）。写满停止，不循环覆盖、不自动擦除。再次上电会扫描已有
页并追加；扫描存量很大时可能超过缓存时长，缺失数据可从序号和时间戳识别。遇到非日志内容停止写入。
掉电可能损失未写出的 RAM 样本及在途页；CRC 不通过的页由解码器剔除。
掉电若连页头都未写完整，启动扫描可能拒绝追加，但不会覆盖原数据。

## 记录内容

- Yaw 目标、实际角、误差、摇杆规划速度、PID 输出、底盘角速度、前馈、
  MIT 合成速度、反馈速度、Kd 和力矩。
- 底盘三轴命令（SI）与三轴规划值（原有抽象量纲）、四轮反馈速度、Yaw 编码器角。
- 两路 IMU 年龄（us，未发布/时间异常为 `UINT32_MAX`）、五个电机协议状态、
  在线/就绪/请求使能、控制模式与诊断故障位。
- 两次采样间累计的规划失败、平移坐标系切换，不会因 100 Hz 采样漏掉 2 ms 瞬态事件。

`flags` 位：0=控制 IMU 有效，1=Yaw 目标已建立，2=云台平移坐标系，
3=本采样间隔规划失败；4..8=四轮/Yaw ready，9..13=online，14..18=请求使能，
19=本间隔坐标系切换，20..23=ChassisMode，24..27=GimbalMode，
28=本间隔曾出现控制 IMU 无效（包含短于 10 ms 的中断）。
电机协议状态在 `motor_states` 的低 20 bit，每路占 4 bit，四轮在前、Yaw 最后。

每页 256 byte，小端头为 `<IHHII>`：`CLG1`、有效记录数（1 或 2）、记录大小（120）、
页号、随后 240 byte 的 CRC32（IEEE）。记录为 `<QII22f4I>`，字段顺序在
[ChassisFlashLog.h](ChassisFlashLog.h) 和 [flash_log.py](flash_log.py) 定义。
`sequence` 每次采样递增，队列满也递增，因此序号缺口表示丢样；每次启动从 0 开始。
每页写前检查空白、写后回读比对，任一失败停止，不复用失败页。

## 状态与导出

调试器可观察 `ChassisFlashLog_Debug`：state 0=初始化，1=就绪，2=记录，
3=写满，4=Flash I/O 错误（含初始化/QE 失败），5=非日志/非空白目标页，6=已停止。
`written_records` 与 `dropped_records` 分别是本次启动成功写出、队列溢出丢弃的样本数；
`next_address` 是历史日志与本次日志占用的总长度。芯片识别失败仍保留系统初始化故障，
记录器失败不撤销原有电机安全逻辑。清空日志需另行显式擦除，不在启动时执行。

故障复现后，用**与板上固件完全匹配**的 ELF 和底盘 OpenOCD TCL 端口导出：

```sh
python3 User_File/Application/Chassis/FlashLog/flash_log.py dump \
  --elf build/ChassisBoard/H7_Framework.elf --port 6666 --output /tmp/chassis-flash.bin
python3 User_File/Application/Chassis/FlashLog/flash_log.py decode \
  /tmp/chassis-flash.bin /tmp/chassis-flash.csv
```

脚本先核对记录器代码，再通过 RAM 邮箱请求停止采样、排空队列、分块读取 Flash。
不发送 halt/reset、不打断点、不调用 MCU 函数；控制任务继续运行。导出会停止本次记录，
重新上电后追加开启下一次记录。导出期间不要重新构建替换所用 ELF、启动另一个导出器
或占用相同调试邮箱。读取失败会报错；不将空/残缺内存读取当成有效数据。

主机模拟已检查：开机无运动也记录、Flash 就绪前缓存、满队列丢样、奇数尾样本排空、旧日志追加、非日志保护、
写满停止、读写失败停止及邮箱回读；模拟生成的记录经 CSV 工具校验 CRC。
实际 OSPI 接线、芯片状态、DMA 时序及实车采样仍需烧录后验证。

Flash 自动轮询超时检查已同步 ISR 与任务的 64 位时间戳读取，并在相减前检查
时间顺序，修复起始时间被 ISR 更新时误报超时的竞态。该修复不掩盖真实超时，
发生传输错误或写入校验失败仍停止。导出脚本使用小块对齐 32 bit 内存访问，避免
DAP 大块字节读取失败；构建后请保留与板上版本匹配的 ELF。
