# OLD21_RESULTS

**状态：NOT EXECUTED — INPUT_UNAVAILABLE**

## 1. 目标与边界（记录，即使未执行）

old21 是**高 Fitness 但历史几何候选仍合理**的条件案例：

- 时刻 517.936350068 s
- Source 约 2,601 点
- 原 CUDA-NDT Fitness ≈ 20.86761
- 原 Align 已找到正确附近候选，A2 条件实验保留约 1.976 m 横向修正

它只适合验证一件事：**在高 Fitness 且候选合理时，GICP 能否保留有合理几何支持的解。**
它与 C4_22 是**不同帧**，本轮没有把两者混为一谈；old21 也**不能**用来证明错误盆地恢复能力。

## 2. 实际检索结果

| 位置 | 内容 | 是否含 old21 Source/Target |
|---|---|---|
| `/media/ltc/新加卷/autoeagle/phase_kg_s3/track_A_geometry_fusion/conditional_old21_scoremap_01/` | `config.frozen.json`、`original_ndt.stderr`、`result.json`、`runtime_identity.json`、`source_snapshot/` | **否**。`source_snapshot/` 只有 `deskew.ldd.txt`、`ndt.ldd.txt`、`score_service.ldd.txt` 与空的 `research/`、`shared/` 目录 |
| `/home/ltc/.codex/visualizations/2026/11/17/01a1e768-…/kg_s3_implementation/` | `coordinator_code`、`track_A_code`、`track_B_code`、`reports` | **否**。全树无 old21 的 `.bin`/`.pcd`；`find -iname "*old21*"` 只命中源码与报告 |
| `phase_e4_local_convergence/runs/*/captures/` | 729 个 capture | **否**。按 old21 时刻换算 stamp = `1763206058786338164`，无此文件 |

因此 old21 的 **2,601 点 Source 与对应 Target 在本机不存在**。

## 3. 为什么不用替代输入

- 用相邻帧 Source 冒充 old21：违反"不允许使用相邻帧替代关键输入"。
- 用 0105 轨迹或 KC-S2 的演化 prior/P：KG-S3 已明确它们属于不同的条件模型，
  不能当作原始捕获先验。
- 用一个新造的 prior 再跑 GICP：那是在测自己造的输入，不是在测 old21。

## 4. 缺口清单（精确到字段）

1. `source_path` / Source 字节（约 2,601 点，格式应为 `<u64 n> + 3×f4`）——**缺失**
2. 该帧 Target（历史捕获的局部地图）——**缺失**
3. 原请求的 `roi` 参数——**缺失**（`config.frozen.json` 只有 bag/audit/extrinsics 路径与时间窗，
   无该帧 ROI）
4. 原 CUDA-NDT Fitness 20.86761 的原始记录——仅存在于 KG-S3 报告文本，**无机器可读原件**

## 5. 因此本轮对 Q4 的回答

**NOT COVERED。** 不是"GICP 在 old21 上失败"，而是**无法在合法输入上评价**。
任何关于 old21 的陈述都只能是 KG-S3 已发布结论的转述，不构成本轮新证据。
