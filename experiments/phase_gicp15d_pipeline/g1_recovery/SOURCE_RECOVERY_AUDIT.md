# SOURCE_RECOVERY_AUDIT — G1-R 输入恢复

只读审计。本轮**没有**新建 map/seed/prior。所有数字来自实际 `sha256sum`、二进制头解析与 CSV 检索。

## 1. C4_22（第一优先级）：可用，但有一个来源差异

**找到的位置**（不在 KG-S3 归档内，而在生产侧历史审计树）：

```
/home/ltc/flyagle/auto-eagle-main/fitness_test/src/a2rl_localization/lidar_ndt/
  audit/ndt_uncertainty_research/phase_e4_local_convergence/
    runs/A1_source_restore/captures/1763205937849981440.{json,bin}   <- 2499 点 Source + 历史 Seed + ROI
    runs/A1_wide35/captures/1763205937849981440.{json,bin}           <- 3832 点 Source + 历史 Seed + ROI
    results/closed_loop/NARROW_A1_35_pass8_frames.csv                <- 该帧的实际 CUDA-NDT 结果
    results/closed_loop/WIDE_A1_35_pass8_frames.csv
```

| 字段 | narrow 变体 | wide 变体 |
|---|---|---|
| header_ns | 1763205937849981440 | 同 |
| t (old relative) | 396.999993344 | 同 |
| Source 点数 | **2,499** | 3,832 |
| Source sha256 | `50ae6316f9d571e032939976d7fdacadd1c469814577b19dcfc942bc9644bc9b` | `58382bf0c83562696d2d7aa0cc2384813584d797136636109afd9fb1b44129e4` |
| 历史 Seed（记录值） | `(-293.2270, -172.1220, -30.7294)` | `(-292.9763, -172.1542, -30.6367)` |
| ROI 中心 | `(-295.4878, -174.6181, -30.7246)`，gen 1042 | `(-295.4816, -174.5785, -30.6367)`，gen 1041 |
| 历史 NDT final | `(-293.7944, -170.2990, -30.4014)` | `(-293.5767, -173.1701, -31.0911)` |
| 历史 NDT fitness | 22.4702 | 16.0811 |
| 历史 NDT exit | OBJECTIVE_CONVERGED | OBJECTIVE_CONVERGED |
| 历史 proxy_error | **0.1305 m** | **2.9079 m** |
| seed_proxy_error | 1.9666 m | 2.0976 m |

`proxy_error` 的 `reference` 列在 CSV 中明确标注为 `PROXY_REFERENCE_NOT_GT`。

### 来源差异（必须声明）

KG-S3 的 `C4_22_wrong_basin_material_identity.json` 记录
`source_sha256 = 964d20dae94d6e6cdffbf37b72a3612db204c99970f9f697d39d8361ce1fbdc3`，
**本地不存在任何该 sha 的文件**（全盘检索 0 命中）。本地同 stamp 的 2,499 点 Source
sha 是 `50ae6316…`，并且它**正是**该帧历史 NDT 结果 CSV 中记录的 `source_sha`。
两者点数相同、header 相同，因此判定为同一帧的**不同序列化/归档副本**，
不是不同帧。本轮使用 CSV 自证的那一份（`50ae6316…`），并同时记录两个 sha。

### Target：冻结字节不可用，按记录合同重建

KG-S3 记录 `target_sha256 = cfff1a6c…`、98213 点。全盘检索 **无该文件**，
`sizes==8+98213*12` 的二进制 **0 命中**，`phase_e4` 的 `lap_targets/` 为空。

本轮用**冻结的 1013 Target**（`map1013_leaf8.bin`，sha `2f7c78d7…`，997,680 点）
按该帧 json 记录的 ROI 重建：圆柱 `r = 120 m`、`z ∈ [cz−10, cz+30]`、中心取 json 的 `region` 中心。

| 变体 | 重建点数 | 重建 sha256 | 历史点数 |
|---|---:|---|---:|
| narrow | 98,216 | `00df79f95d5f…` | 98,213 |
| wide | 98,214 | `27a7ca02e55b…` | 98,213 |

差 3 / 1 点（0.003%），归因于历史裁剪所用的全局云快照与当前冻结 1013 云不完全同一版本。
**这是重建，不是历史字节。** 报告中每个 C4_22 结果都带 `TARGET_RECONSTRUCTED` 标记。

### 本轮不需要、也确实没有成熟 15D Prior/P

G1 是纯几何配准：只消费 Source / Target / Seed。KG-S3 记录的
`original_15d_prior_P_available: false` 对 G1 **不构成阻断**，本轮未使用任何 prior/P，
也未构造替代。

## 2. 5925 / 5931：输入部分可用，本轮未执行（PARTIALLY_AVAILABLE）

历史基线在 `analyzer/lidar_ndt/audits/gicp_vgicp_benchmark/runs/20261015_actual/`，
使用 `map_1224.pcd`（sha `976af9e2…`）与另一条 bag（stamp `1791985…`）。

| 项 | 5925 (`1791985924999999232`) | 5931 (`1791985931000001280`) |
|---|---|---|
| 冻结 Source PCD | **存在** `frozen_sources/source_1791985924999999232.pcd`（及 `fusion_…`） | **存在** `source_1791985931000001280.pcd` |
| Seed | `lap_inputs/seeds.json` 中**无该精确 stamp**，最近为 `1791985924999031899`（早 0.967 ms） | **无**：`seeds.json` 最后一个 stamp 是 `1791985925989037677`，比 5931 早 **5.011 s** |
| 冻结 Target | `lap_targets/` 为空 → 需从 `map_1224.pcd` 运行时裁剪；本模块的加载器只读 `.bin`，不含 PCL | 同 |
| 历史结果 | `SEED_SENSITIVITY.csv` 有该帧 small_gicp/FastVGICP/CUDA-NDT 全量行 | 同 |

**结论**：5931 的 Seed 缺失；5925 的 Seed 只有 0.97 ms 邻值。
本轮**未执行**两例，也没有用邻值 seed 冒充历史 seed 后宣称复现。
精确缺口：`lap_inputs/seeds.json` 的覆盖区间终止于
`1791985925989037677`，且 `lap_targets/` 为空。

## 3. old21：INPUT_UNAVAILABLE

KW-S3 的 `conditional_old21_scoremap_01/source_snapshot/` 只有 `*.ldd.txt` 与空的
`research/`、`shared/` 目录；编排工作区
`/home/ltc/.codex/visualizations/2026/11/17/01a1e768-…/kg_s3_implementation/` 内
**没有任何 old21 的 Source/Target**。

按 old21 时刻 517.936350068 s 换算 stamp = `1763206058786338164`，
`phase_e4` 全部 run 的 `captures/` 中**无此 stamp**（729 个 capture 文件均不含）。

缺口：**old21 的 2601 点 Source 与其 Target 在本地不存在**。
因此本轮**没有**执行 old21，也没有用相邻帧替代。

## 4. 上轮 G1 产物：直接继承，不重跑

`G1_VALIDATION_RESULTS.md`、`G1_FAILURE_ANALYSIS.md`、`AUDIT_ARCHITECTURE_COMPARISON.md`、
`ARCHITECTURE_DECISION.md`、`REPRODUCE.md`、`results/*` 与
`/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/artifacts/*` 全部为
**INHERITED_EVIDENCE**。本轮未重新运行 1,648 次配准。
