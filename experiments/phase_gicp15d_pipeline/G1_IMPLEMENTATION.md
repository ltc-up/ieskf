# G1_IMPLEMENTATION

本轮新增代码：`src/gicp15d/`。全部 C++17；依赖只有系统 Eigen 3.4、OpenMP、本地
`small_gicp fa0cfc98` 头文件、`nlohmann/json`（仅 CLI 解析）。**不依赖 PCL / ROS / CUDA。**
独立 `CMakeLists.txt`，不进入生产 `colcon`，不修改生产仓库。

## 1. 构建

```sh
cd /home/ltc/flyagle/ieskf-gicp15d/src/gicp15d
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/gicp15d_tests          # 41 checks
```

产物：`build/gicp15d_frontend`（验证入口）、`build/gicp15d_tests`（单元/合同测试）。

## 2. 模块与语义

### `point_cloud_io.{h,cpp}`

- `loadXyzBin`：`<u64 n>` + `n×3 float32`，与 `map_target.bin` / `map1013_leaf8.bin` /
  `<stamp>_A1.bin` 完全一致；20M 点安全上限；非有限点直接失败。
- `fileSha256`：自带 FIPS-180-4 实现，不引入 OpenSSL 依赖。
- `voxelDownsample`：仅用于 coarse 通道。
- `cloudStats`：边界/均值/非有限计数。

### `local_covariance.{h,cpp}`

两种模式：

- `SMALL_GICP_UNIT`：与上游 `estimate_local_features<CovarianceSetter>` 精确一致
  （kNN 含自身、`cov=(Σppᵀ − mean·Σpᵀ)/n` 有偏、特征向量保留、特征值替换 `(1e-3,1,1)`、
  邻居<5 → Identity）。
- `SPECTRUM_FLOOR`：`λ_i ← max(λ_i, ρ·λ_max)`，保留空间尺度。

同时写 `normal`。输出 `CovarianceStats`（中位 λ_max/λ_min、有效 floor、条件数、邻居不足计数）。

### `global_map_manager.{h,cpp}`

- `load()`：按路径 + **内容 sha256** 双重判定缓存（路径相同但内容变了不会命中）。
- `crop()`：按 `(target sha, region.generation)` 缓存 ROI 子点云；子点云复制
  **点/法向/已算好的协方差**，只重建 kd-tree。
  → 这是对 AW "全局 NN 后 ROI 剔除、不重试下一邻" 缺陷的修正。
- ROI 命中不足 100 点时**失败退出**，绝不回退全图。

### `correspondence_rejector.h`

`GicpRejector`：距离门（平方，m²）→ ROI 门（圆柱 + Z 带）→ 互惠检查（把目标点反变换回
源坐标系，要求其最近源点就是当前源点）。三类拒绝各自计数，`mutable` 以适配
`reduction.linearize` 的 const 签名。

### `robust_gicp_factor.h`

`RobustGICPFactor`：残余、`(C_t + T C_s Tᵀ)^{-1}`、`J`、`H=JᵀMJ`、`b=JᵀMr`、`e=½rᵀMr`
与上游逐项相同；额外

- 法向一致性门：`n_t · (R n_s) < cos(θ_max)` 直接拒绝；
- IRLS 权重 `w(r)`（`r` = **米制**欧氏残差）：Huber `min(1, c/r)`、Geman-McClure `(c²/(c²+r²))²`，
  乘到 `H,b,e` 上。

这是 FAR-LIO 的做法（核在因子内），但尺度用米而不是上游未启用的 `sqrt(2e)` 单位。

### `registration_diagnostics.{h,cpp}`

`computeDiagnostics`：在**返回的最终位姿**上用调用方自有的
`std::vector<RobustGICPFactor>` 重新跑一遍 `ParallelReductionOMP::linearize`，
因此 NN 搜索、Mahalanobis、法向、权重全部是最终位姿的，而不是优化器上一次迭代的
（上游 `RegistrationResult.H/b` 属于**步进前**位姿，直接用会错位）。

输出：`pairs / candidate_pairs / 三类拒绝计数 / inlier 比`、
欧氏与 Mahalanobis 残差的 mean/median/p95/max、IRLS 权重均值与最小值、
12 扇区直方图（body 系方位角）、占用扇区数、主导扇区比例、归一化熵、距离中位/p95、近场比例、
6×6 H 与谱、条件数、最弱平移方向与比值、`rank_deficient`。
`covariance_status` 恒为 `"NOT_A_COVARIANCE"`。

### `gicp_frontend.{h,cpp}`

`align(request) -> RegistrationOutput`。流程：载入并校验 Target → 可选 ROI 裁树 →
载入并校验 Source → Source 协方差 → 可选 coarse 通过 → 高级 pass →
最终位姿诊断 → 可选多 Seed 探针。所有结果保留 `source_sha256 / target_sha256 / preset /
config_json / offset_s / corpus`，身份不丢。

命名 preset（每个 = G0 设置 + 单一机制，便于消融）：

| preset | 与 G0 的唯一差异 |
|---|---|
| `G0_AW_EQUIV` | —（AW 的 50 iter / 1 mm / 0.1° / 4 m / kNN20 / UNIT 协方差） |
| `G0_SMALL_GICP_NATIVE` | 上游默认 20 iter（"有没有动过求解器设置"的对照） |
| `G1_SPECTRUM` | `SPECTRUM_FLOOR` 协方差 |
| `G1_ROI_CROP` | ROI 裁树 |
| `G1_NORMAL_GATE` | 法向一致性 60° |
| `G1_RECIPROCAL` | 互惠检查 |
| `G1_ROBUST_GM` / `G1_ROBUST_HUBER` | 鲁棒核，`c=1 m` |
| `G1_COARSE` | coarse 1.5 m → fine |
| `G1_FULL` | 以上五项同时开启 |

### `frontend_main.cpp`

```sh
gicp15d_frontend --cases cases.jsonl --out results.jsonl \
                 --presets G0_AW_EQUIV,G1_FULL --summary summary.json \
                 [--threads N] [--probe] [--probe-offsets 0,2,5,8,12] [--limit N]
```

逐帧求解全在 C++ 内；Python 只在 `tools/` 里做案例抽取与统计。

## 3. 测试（`tests/run_tests.cpp`，41 项断言，全通过）

| 组 | 覆盖 |
|---|---|
| 点云往返 | 写入/读取一致、sha 长度、缺失文件返回空 |
| 协方差模式 | 无邻居不足；UNIT 的谱精确为 `(1e-3,1,1)`；FLOOR 的 `λ_min ≥ ρλ_max` 逐点成立；条件数>1 |
| Rejector | 距离门内/外、ROI 内/外、计数正确 |
| 鲁棒权重 | NONE=1；Huber 内 1 / 外 `c/r`；GM 在 `r=0`为 1、`r=c`为 0.25、单调递减 |
| 信息诊断 | 条件数=100、特征值升序、非秩亏、最弱平移比=0.1、显式非协方差标签；零 H 报秩亏 |
| 端到端 | 合成走廊（带抖动 + 斜挡板破除平移对称）从 0.35 m / 1.5° 扰动恢复到 **0.0000 m / 0.00000 rad**，5,117 内点 |
| 确定性 | 单线程两次 align 逐位相同 |
| 失败模式 | 缺输入→`TARGET_LOAD_FAILED`；目标 sha 不符→`TARGET_SHA256_MISMATCH`；空 ROI→`TARGET_ROI_FAILED`（**不回退全图**） |

## 4. 未实现（诚实边界）

- 无 GPU 路径（本轮只做 CPU）。
- 无增量/滑动地图（Target 视为静态）。
- 无参数扫描（按任务书，禁止看到参考误差后调参）。
- 无 15D 融合（G2 未实施）。
- `SPECTRUM_FLOOR` 只测 `ρ=1e-3`；`robust_scale_adaptive` 字段存在但**未启用**。
