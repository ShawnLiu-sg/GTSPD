# 同站起降卡车与无人机联合配送求解器

C++17 实现论文第 5 节的混合自适应大邻域搜索。卡车从仓库出发并携带全部包裹。无人机在许可的卡车停靠点发射和回收，每架无人机在同一站至多执行一个闭合架次。程序按航段剩余载荷计算电耗，并按每站最长架次计算卡车等待时间。

## 运行示例

在解压目录的 PowerShell 中运行预编译的 Windows x64 程序。

```powershell
.\truck_drone.exe example.txt
.\truck_drone.exe example.txt --plain
```

`--plain` 关闭精确邻域，其余搜索设置沿用输入文件。程序向标准输出写入 JSON，包含卡车路线、每条闭合架次、能耗、等待时间、排放、成本分项和搜索计数。`exact_completed` 计数对应在时间内完成枚举、动态规划及方案重建的局部调用。`exact_improvements` 统计完整目标下降的局部调用。最终目标值来自全局启发式搜索的最佳可行解。

## 从源代码构建

在 Visual Studio 的 x64 Native Tools Command Prompt 中执行。

```cmd
cl /nologo /std:c++17 /EHsc /O2 /MT /Fe:truck_drone.exe main.cpp search.cpp model.cpp exact.cpp
```

也可用 CMake 构建。

```powershell
cmake -S . -B build
cmake --build build --config Release
```

## 实例输入

以 `example.txt` 为模板。首行为 `STDRP 1`，随后按表中顺序填写段名和值。仓库编号为 `0`，客户编号为 `1` 到 `n`。

| 段名 | 值的顺序 |
|---|---|
| `CUSTOMERS` | 客户数 `n` |
| `TRUCK` | 载重 kg、速度 m/s、空载油耗率 L/s、每公斤载荷增量油耗率 L/(s·kg) |
| `DRONE` | 无人机数、载重 kg、单架次可用电量 kWh、速度 m/s、空载功率 W、每公斤载荷增量功率 W/kg |
| `COSTS` | 卡车每米运营成本、无人机每米运营成本、每 kg CO₂e 的碳成本、每秒等待成本、燃油排放因子 kg CO₂e/L、电池端电耗的排放因子 kg CO₂e/kWh |
| `ORDERS` | `n` 行，每行依次为客户编号、包裹质量 kg、可由无人机服务标记、允许发射标记。标记取 `0` 或 `1` |
| `COORDINATES` 或 `ROAD` | `COORDINATES` 后填 `n+1` 行 `编号 x y`，坐标以米计。由曼哈顿距离生成卡车矩阵，由欧氏距离生成无人机矩阵。也可在 `ROAD` 后直接填 `(n+1)×(n+1)` 有向道路距离矩阵，随后用 `AIR` 给出同尺寸有向飞行距离矩阵，单位均为米 |
| `SEARCH` | 随机种子、迭代上限、全程时间预算 ms |
| `DESTROY` | 最小移除数量、最大移除数量、Sigmoid 斜率与中点 |
| `ANNEAL` | 初始温度、每步冷却倍率、最低温度 |
| `ADAPT` | 权重更新周期、更新系数、插入噪声系数，以及四档奖励，依次对应全局改进、当前改进、其他接受和拒绝 |
| `EXACT` | 调用周期、客户池大小上限（1–16）、单次调用预算 ms。周期为 `0` 时运行普通 ALNS |
| `END` | 输入结束 |

样例的客户位置和包裹为合成数据，搜索参数为演示设置。无人机功率采用公开 Matrice 600 Pro 试飞均值在 11.5 m/s 下的插值拟合 (1179.31+184.54q) W。样例的 0.510 kWh 是按源研究示例中 0.600 kWh 满充电量和 15% 留存电量计算的可用电量。 [公开试飞数据和参数](https://www.osti.gov/pages/servlets/purl/2583936)
