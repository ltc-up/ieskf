# HISTORICAL_5925_5931_RESULTS

**状态：NOT EXECUTED — INPUT_PARTIALLY_AVAILABLE**

本轮**没有**运行 5925 / 5931。原因不是方法不可行，而是**关键输入字段缺失**，
且任务书禁止用邻帧或近似 seed 冒充历史 seed 后宣称复现。

## 1. 已核实的可用部分

历史基线：`analyzer/lidar_ndt/audits/gicp_vgicp_benchmark/runs/20261015_actual/`

- 配置：`lap_inputs/manifest.json` →
  `config_id = A_rebuilt_shared080_knn20_t4_cuda_20261015`，
  `covariance_knn 20`、`num_threads 4`、`max_iterations 50`、
  `max_correspondence_distance 4.0`、`translation_epsilon 1e-3`、
  `rotation_epsilon 1.745e-3`、`voxel_resolution 1.0`、`neighbor_search DIRECT7`。
  **与本轮 `G0_AW_EQUIV` 的求解器设置完全一致**（这是唯一一件顺利的事）。
- 地图：`map_1224.pcd`，sha `976af9e2ffc3ba28fa6741e56b93d209988952b64674c7572a22caa8714d4828`。
- 冻结 Source PCD **存在**：
  `frozen_sources/source_1791985924999999232.pcd`（5925）、
  `frozen_sources/source_1791985931000001280.pcd`（5931），
  另有对应 `fusion_*.pcd`。
- 历史结果行完整保留：`SEED_SENSITIVITY.csv` 对 5925 同时给了
  `small_gicp_gicp`、`fast_vgicp_cpu`、`cuda_ndt` 三方法的最终矩阵、inliers、exit。

## 2. 精确缺口

| 缺口 | 证据 |
|---|---|
| **5931 无 Seed** | `lap_inputs/seeds.json` 共 17,680 条，stamp 区间 `1791985749199066961 … 1791985925989037677`；5931 的 stamp `1791985931000001280` **超出末端 5.011 s** |
| **5925 无精确 Seed** | 最近条目 `1791985924999031899`，早 **0.967 ms**。历史运行本身用的也是这个邻值（`SEED_SENSITIVITY.csv` 第 2 列即该 stamp），但那是历史实验的选择，本轮不能替它做 |
| **无冻结 Target** | `lap_targets/` 目录**为空**（0 文件）；Target 需在运行时从 667 MB 的 `map_1224.pcd` 裁剪 |
| **本模块无 PCD 读取能力** | `src/gicp15d` 的加载器只读 `<u64 n> + 3×f4`，刻意不引入 PCL。要读 `map_1224.pcd` 必须新增 PCL 依赖或自写 PCD 解析 |

## 3. 为什么没有用近似输入强行跑

- 用 0.967 ms 前的 seed 跑 5925，属于**替换历史 Seed**；用任意 seed 跑 5931，
  属于**构造 Seed**。两者都会让"GICP 优势是否复现"这个问题失去意义。
- 从 `map_1224.pcd` 重建 Target 需要先确定历史 ROI 半径。历史报告同时提到
  120 m 生产圆柱与后续 250 m 尾部实验，本轮无法在无原脚本的情况下确定该帧用的是哪一个。
  猜错会直接改变结论方向。
- 因此按任务书 §三.B：**明确写 INPUT_UNAVAILABLE / NOT EXECUTED，不伪造**。

## 4. 历史结论（INHERITED_EVIDENCE，未由本轮复验）

来自 `gicp_vgicp_benchmark/BAD_SEED_REPORT.md` 与 `FINAL_COMPARISON.md`：

- 5925：`small_gicp` 恢复到 Y≈533.175（XY 误差 0.655 m），CUDA-NDT 收敛到 525.909
  的错误解（XY 7.459 m）；FastVGICP 1 m 停在 528.078 且未收敛，2 m 配置恢复到 533.143。
- 5931：三种方法从原错误 EKF 初值**全部失败**。
- 同批还有 small_gicp 的反例：5926.949997312 帧以 10.048 m 误差 `converged = true`。

**这些数字本轮没有复跑，因此只能作为历史记录引用，不能作为"当前 C++ 前端可复现"的证据。**

## 5. 若要执行，最小前置条件

1. 确认 5925/5931 使用的 ROI 半径（120 m 还是 250 m）与 Target 生成脚本；
2. 取得 5931 的原始 Seed（或明确授权使用同帧最近值的规则）；
3. 为本模块增加只读 PCD 输入（或预先把 Target 转成 `.bin` 并冻结 sha）。
满足其一后，用 `G0_AW_EQUIV` 单配置即可完成 Q2，不需要 8 套预设。
