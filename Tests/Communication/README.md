# 通信底层审查与主机回归（2026-09-25）

本批限定于 CAN/FDCAN、USB、OSPI 的接收边界、异步缓冲所有权和提交失败传播；同时核验 SPI/UART 的现有保护与 DMA 布局。不是全仓或全部并发时序审计，也不是实机验收。

## 基线与责任归属

- 本地 main：`95b55e37892fa9d69c0e4d7003ecd115f07a59ee`。
- GitHub main 已获取的远端快照：`56f23d0e87f46cfcc6783ca3bd456e4e8247adf9`。
- 上游 RoboMaster_H7 已获取的远端快照：`fac097704ce13986f244d0a854814e04ef005c87`。
- 修改前，本批七个 BSP/USB 桥接文件与 main、GitHub main 内容一致（比较时统一 CRLF/LF）。USB/OSPI 与上游快照也完全一致；上游 CAN 的接收数组、远程帧配置和长度分发同样存在下面的问题，其他实现有差异。
- 没有整体合并远端分支。原有 BMI088、DM、Flash 初始化修复保留；Flash 的提交失败处理是在原有修复之上追加的。
- 缺陷位于通用 BSP、CDC 桥接或 Flash 驱动的接口契约，与兵种应用参数、PID 调参或接线无关。HAL 返回失败、合法 CAN 帧类型以及 USB 重枚举均是框架应处理的输入。

本次审查的原始文件快照、逐文件 SHA-256、修改前失败日志保存在本机审查目录：
`C:/Users/FAR/Documents/ChatGPT/H7BSP/Validation/Communication_Audit_20260925/`。
其中 `baseline.json` 固定上述版本与内容对照；`before/` 是修改前的七个源文件，未复制 Git 元数据或凭据。

## 已确认并修复

| 问题 | main 中的触发与证据 | 修复与回归 |
| --- | --- | --- |
| CAN 接收栈越界、长度语义错误 | Classic CAN 的 DLC 9..15 表示 8 数据字节；本仓 HAL `HAL_FDCAN_GetRxMessage` 不区分 FDF，按 `DLCtoBytes` 复制 12..64 字节。`HAL_FDCAN_RxFifo0Callback` 仅提供 8 字节数组，并把原始 DLC 当长度传给设备。标准帧全接收配置可达。 | HAL 接收临时数组扩为 64 字节；只向设备交付标准 Classic 数据帧，并将 DLC 9..15 归一为 8 字节。`classic_dlc` 覆盖 9..15；`storage` 在无设备回调时检查函数返回，旧源码触发栈保护 `0xc0000409`，修复后通过。 |
| CAN 远程帧被当成设备反馈 | 全局过滤使用 `FDCAN_FILTER_REMOTE`，标准 mask=0 接受匹配远程帧；BSP 不检查 `RxFrameType`，同 ID 的 RTR 帧被送入仅有 data/len 的设备回调。远程帧没有对应反馈载荷。 | 硬件配置改为拒绝远程帧，并在分发前检查 ID 类型、数据帧类型、Classic 格式。`remote`、`filter` 修改前失败，修改后通过；普通帧和跨总线隔离继续通过。 |
| USB 重连/启动期间交付错误缓冲 | `CDC_Init_HS` 每次枚举都把 RX 指针换回 `UserRxBufferHS`；`CDC_Receive_HS` 丢弃实际 `Buf`，BSP 使用旧 Active 指针。启动完成前的重装接收也会造成同样不一致。 | CDC 桥接传入实际 `Buf`，BSP 据本包指针交付并选择下一缓冲。测试直接通过生产 `USBD_Interface_fops_HS` 执行 Init/DeInit/Receive，覆盖正常双缓冲、重新枚举和启动早包。 |
| USB 异步发送被下一帧覆盖 | `USBD_CDC_SetTxBuffer` 保存原指针；PCD 在 FIFO 空中断中继续读取。EricTool 每次 `Output()` 先清空并重写同一个发送数组，即使后续发送返回 BUSY，在途帧也已被破坏。关闭 USB DMA 不会使发送变成同步复制。 | BSP 在空闲时复制到自有 512 字节 TX 数组；临界区覆盖类句柄检查、Busy 判断、复制和提交，防止断连或另一发送者插入。忙时立即返回 BUSY，保留在途数据；原缓冲返回后可改。测试覆盖源缓冲重写、忙时二次提交、完成后再发及 PRIMASK 恢复。 |
| OSPI 提交失败仍继续下一阶段，Flash 返回假成功 | 原 BSP 忽略命令、DMA、轮询返回值；命令失败后仍调用 DMA。Flash `Get_Buffer`/写使能等先置 Busy，然后无条件返回 true。HAL 的命令配置与间接 DMA 有明确状态前置条件。 | OSPI 返回 `HAL_StatusTypeDef`，命令失败立即停止；Flash 检查每个提交阶段，失败清除软件忙/写使能/抑制轮询标志、计错并向 bool 调用者返回 false。异常提交不再进入下一阶段。覆盖 ERROR/BUSY/TIMEOUT、DMA 拒绝、轮询拒绝、正常路由、Flash 读/写/擦拒绝。 |

同一 OSPI 链路还做了必要的接口收口：

- DMA 接口验证数据阶段、非零长度及 512 字节容量。非法参数测试是防御性回归，不据此声称现有 Flash 调用已经传入超长数据。
- `Enable_Quad_Mode` 的手动 WIP 读取原来从默认命令继承 `DATA_NONE`，无法形成接收 DMA 所需的 `CMD_CFG` 状态；补齐单线数据阶段。正常 Quad 命令序列与提交失败提前停止均有回归。主机模型不模拟真实 Flash 时序。
- 旧 `OSPI_Command_Transmit_Receive_Data` 会在启动 TX DMA 后立即启动 RX DMA，违反同一指令单数据方向约束。当前 `User_File` 源码中没有调用者，改为无副作用地返回 `HAL_ERROR`；使用者必须在完成回调后另发读命令。本项不计作已影响现有应用的故障。
- `HAL_OK` 仍仅代表提交成功。DMA 接受后的硬件异常、超时 Abort、并发 Flash 访问和迟到回调恢复不属于本批完整验证范围，不能用本次结果声称已解决。

## 资料依据

- [Bosch M_CAN 用户手册 v3.3.1](https://www.bosch-semiconductors.com/media/ip_modules/pdf_2/m_can/mcan_users_manual_v331.pdf)：§2.3.21 GFC 的 RRFS/RRFE；§2.4.2 RX 元素的 RTR、FDF、DLC，PDF 第 37、57、58 页。DLC 9..15 的 Classic/FD 含义不同。
- [ST 官方 FDCAN HAL](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/master/Src/stm32h7xx_hal_fdcan.c)：`HAL_FDCAN_GetRxMessage`、`DLCtoBytes`。已对照工程内同名 HAL 的实际复制循环；没有修改 vendor 源码。
- [ST 官方 CDC 类实现](https://github.com/STMicroelectronics/stm32-mw-usb-device/blob/master/Class/CDC/Src/usbd_cdc.c)：`USBD_CDC_DataOut` 把本包 `RxBuffer` 交给 Receive，SetTxBuffer 保存指针，TransmitPacket 启动异步传输；并对照了工程内 PCD 的 `PCD_WriteEmptyTxFifo`。
- [ST 官方 OSPI HAL](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/master/Src/stm32h7xx_hal_ospi.c)：`HAL_OSPI_Command` 的数据阶段/状态更新，以及 Transmit_DMA、Receive_DMA、AutoPolling_IT 的状态约束和失败返回。以工程内 HAL 实现为最终代码事实。

## 排除项与限制

- 当前 HAL 的 `FDCAN_DLC_BYTES_8` 是未左移的 `8`，发送 `DataLength = len` 对 1..8 正确，不能按旧版 HAL 习惯盲目左移 16 位。
- 当前 UART TX 已有自有缓冲、提交锁及 HAL/DMA Busy 检查；Boundary 的 UART TX 回归通过。新 Debug map 中 UART 管理对象段为 `0x24001ce0..0x240048a0`，位于 MPU 的 64 KiB 非缓存 AXI DMA 区，原有 DTCM TX 旧项不能继续标成未修。
- SPI 有事务占用、长度限制、自有 DMA 缓冲和失败释放；SPI6 使用阻塞 HAL，避免 BDMA 访问 AXI。此次审阅路径未确认新增缺陷，不代表所有错误中断、Abort 或并发组合均安全。
- CAN TX 队首失败保留、周期最新值和版本确认是当前接口设计。AutoRetransmission 关闭是策略配置，尚不能据此单独定性为框架 bug；没有擅自切换三路硬件重传。
- CodeGraph 用于定位；CBM 代次仍为 2026-09-04，覆盖检查提示 metadata_changed/partial，影响结果也含旧 CAN 符号。已降级为当前源码、调用点搜索、差异与编译验证；不以旧图证明穷尽性。
- 待板测：真实 CAN DLC/RTR 注入及中断栈余量、USB 连续重连/主机背压/关中断时长、Flash 真实读写擦除与断线恢复、UART 持续发送。未烧录、未上板。

## 运行与结果

独立主机工程不加载 ARM 工具链：

```powershell
cmake -S Tests/Communication -B build/Communication -G Ninja -DCMAKE_C_COMPILER=C:/BSP/mingw64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/BSP/mingw64/bin/g++.exe
cmake --build build/Communication -j 4
ctest --test-dir build/Communication --output-on-failure --timeout 10
cmake --build build/Initialization -j 4
ctest --test-dir build/Initialization --output-on-failure
cmake --build build/Boundary -j 4
ctest --test-dir build/Boundary --output-on-failure
cmake --build --preset Debug -j 4
```

测试直接编译完整生产 BSP 源文件及 CDC C 桥接；替身只模拟 HAL、USB 类与 RTOS 服务。CAN 替身严格复现本仓 HAL 的 DLC 复制长度，GNU 栈保护检测越界；USB 模型保留发送指针以模拟异步所有权。它们不是总线、电气、USB 枚举或设备时序仿真。

修改前保留了首批 12 项中的 9 项失败、补充 3 项 CAN 失败（含栈保护）及 Flash 假成功失败日志。新增返回值回归在修复后执行。最终 Communication **17/17**、Initialization **7/7**、Boundary **5/5** 通过，Debug 固件链接成功，`git diff --check` 通过。

新固件 DTCMRAM 使用 107016/131072 字节；DMA 区使用 20704/65536 字节。编译仍有既有的未用参数、聚合初始化等警告，未宣称零警告构建。

## 分步提交验证

按用户要求，将达妙应答、板载初始化、OSPI 失败传播、CAN 接收和 USB 缓冲所有权分别提交到 main，文档单独提交。Flash 与测试中的跨批次内容通过暂存区拆分，工作区保留最终实现。

每个代码提交均从当时的暂存区导出独立源码快照，使用该快照运行对应主机回归及 Debug 固件构建；不会使用工作区中尚未进入该提交的修复来证明中间提交可用。分步验证日志保存在本机 `Validation/Commit_Split_20260925/` 审查目录。尚未推送或进行板上验证。
