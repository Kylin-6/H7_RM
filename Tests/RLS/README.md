# 定长 RLS 主机测试

直接编译正式的 `alg_rls.cpp`，不依赖 H7 硬件。算法最多辨识 6 个参数，输入为同一时刻的特征向量 `features` 和标量测量 `measurement`，模型为 `measurement ≈ featuresᵀ × parameters`。调用方应自行保证特征的物理定义、单位、激励覆盖和测量时序一致。

`Init(dimension, forgetting_factor, initial_covariance, initial_parameters)` 要求维度在 1～6、`0 < forgetting_factor <= 1`、初始协方差大于零；初始参数可省略，默认全零。参数和协方差内部使用 `double`，无动态内存。`Update` 对非有限输入、数值失败或候选协方差失去正定性返回 `false`，保留全部旧状态。`Reset` 恢复最近一次成功 `Init` 的参数和对角协方差。`Get_Parameters` 要求输出容量不小于维度；无效索引的 `Get_Parameter` 返回零。

遗忘因子等于 1 时保留全部历史观测；小于 1 时旧数据权重随时间按其幂次衰减，更快跟随参数变化，也更容易受测量噪声影响。无新信息的更新仍会使协方差除以遗忘因子；极端参数导致数值溢出时更新失败，应检查返回值并根据应用需要复位。

```powershell
cmake -S Tests/RLS -B build/Tests_RLS -G Ninja -DCMAKE_CXX_COMPILER=C:/BSP/mingw64/bin/g++.exe -DCMAKE_BUILD_TYPE=Release
cmake --build build/Tests_RLS
ctest --test-dir build/Tests_RLS --output-on-failure
```

测试覆盖输入契约与失败状态不变、二维单步解析解、六维可复现收敛、遗忘因子对变化模型的适应、数值溢出后复位，以及合法极端输入导致的非正定候选拒绝。测试证明算法源码在主机上符合这些契约；具体电机模型还需实际数据辨识与板上运行时间验证。
