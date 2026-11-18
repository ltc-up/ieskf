# EKF_SEED_PROVENANCE

## 1. 本轮使用的 Seed 是什么

`/media/ltc/新加卷/autoeagle/phase_bg_far_lio/c4/frozen/requests.json` 有 35 条请求，
每条含：

```
case_id, header, group, scene, source_path, target_path, target_sha,
seed_common (4x4), roi, q, max_iterations, bd_config, identity
```

- `seed_common` 是**该帧 CUDA-NDT 匹配请求里实际使用的初值**，即
  `T_odom_native`（map ← base_link）。
- 该目录同时保存了每帧的 `source.bin` 与 `target.bin`，其 sha 与 `target_sha` 一致；
  例：C4_22 的 `source.bin` sha `964d20da…`、2499 点；`target.bin` sha `cfff1a6c…`、98213 点。

**因此本轮的主 Seed 是"历史匹配请求中的 Seed"，不是 NDT 匹配后的 Final。**
`results.jsonl` 与 `WALL_frames.csv` 中的 `ndt_x/ndt_y` 只作为**离线对照**，从未回流为 Seed。

## 2. 三个位姿严格分开

| 符号 | 来源 | 本轮角色 |
|---|---|---|
| `EKF_PREDICTED_SEED` | `requests.json:seed_common` | **唯一主实验 Seed** |
| `NDT_MATCHED_POSE` | `WALL_frames.csv:ndt_x/ndt_y` | 离线对照 |
| `EKF_POST_UPDATE_POSE` | 未使用 | 未参与本实验 |

## 3. 来源强度：`SEED_PROVENANCE_PARTIAL`

可以确认：

- Seed 是**匹配请求里被实际提交的初值**（与 matcher 协议字段同名 `seed_common`，
  与 Phase AR/AS 的 `transactions.jsonl.seed` 定义一致）；
- 同一帧的 Source/Target 与 Seed 出自同一份冻结请求，坐标系一致（map ← base_link）；
- Seed 随帧单调漂移：C4_18 0.446 m → C4_25 2.598 m（相对工程代理），
  这与"EKF 观测不足后预测逐渐偏离"的形态一致。

**不能确认**（因此标注 PARTIAL）：

- 该 Seed 是否**逐帧**来自当帧的 EKF 预测，而不是由某次离线回放重建。
  冻结目录没有携带逐帧的 15×15 P 或 IMU 传播记录；
- `phase_koide_geometry_s2/inputs/old_conditional_window.json` 对同源帧的
  `qualification` 明确写：`conditional offline5scan graph, not recovered mature
  historical state; zeroBias/defaultP/initial lateral velocity assumptions explicit`。
  即该系列**不是**成熟的 15D 历史状态。

结论：Seed 的**时间点与语义**成立（匹配前预测），但其**状态完整性**不足，
因此本轮只做"冻结 Seed 的反事实配准"，不做任何 15D 断言。

## 4. 时间线（8 个有代理参考的帧）

`offset = t`（该 bag 的老相对时间基准，与 `WALL_frames.csv` 的 `t` 列同源）。

| frame | t (s) | header (ns) | Seed→代理 (m) | NDT→代理 (m) | 判读 |
|---|---|---:|---:|---:|---|
| C4_18 | 396.60 | 1763205936600030208 | 0.446 | 0.084 | Seed 尚好，NDT 正常 |
| C4_19 | 396.85 | 1763205936850038272 | 0.626 | 0.111 | 同上 |
| C4_20 | 397.35 | 1763205937350027776 | 1.454 | 0.071 | Seed 开始漂移，NDT 仍正常 |
| C4_21 | 397.80 | 1763205937799989248 | 2.117 | 0.109 | Seed 已漂移 2.1 m，NDT 仍正常 |
| C4_22 | 396.999993 | 1763205937849981440 | 2.124 | **4.503** | **NDT 首次进入错误分支**，`reject=True` |
| C4_23 | 397.05 | 1763205937899980032 | 2.152 | **4.342** | 错误分支持续 |
| C4_24 | 397.15 | 1763205937999988224 | 2.249 | **4.308** | 错误分支持续（第 3 帧） |
| C4_25 | 397.40 | 1763205938249996800 | 2.598 | 0.226 | NDT 自行回到正常 |

注意：`t` 列在同一行内出现 396.999993 与 397.35 交错，说明 `t` 是**每次运行自己的**
相对秒，不是严格单调的帧序。**帧序应以 header 为准**（本表已按 header 排序）。
这是上一轮把 396.999993 当作"帧 22 的时刻"并据此排序的口径错误来源之一。

另：`WALL_frames.csv` 该行 `target_points = 98223`，而冻结 `target.bin` 为 **98213** 点，
`map_version = 1042` 一致。两次捕获的 Target 差 10 点，属同一 ROI 的不同快照。

## 5. 与上一轮 G1-R 的口径更正

上一轮 G1-R 使用了：

- **错误的代理**：把 phase_e4 narrow 的 NDT final `(-293.794, -170.299)` 当成代理。
  真实代理是 `WALL_frames.csv` 的 `ndt_x_good/ndt_y_good` = `(-293.924, -170.283)`。
  两者相差约 0.15 m，且 narrow final 本身是另一次 NDT 运行的结果，不是参考。
- **不必要的 Target 重建**：当时判定"98213 点 Target 字节不可用"，实际上它就在
  `phase_bg_far_lio/c4/frozen/22/target.bin`，sha `cfff1a6c…`，与 KG-S3 记录**逐字节一致**。
- **不同的 Source 序列化**：当时用了 `phase_e4` 的 2499 点文件（sha `50ae6316…`）；
  真正的历史 Source 是 `phase_bg_far_lio/c4/frozen/22/source.bin`（sha `964d20da…`，同为 2499 点）。

本轮全部改用真实字节。G1-R 的 C4_22 数字（0.331 / 0.969 m）**不应再被引用**；
以本轮 `G1_FROBENIUS_RESULTS.md` 的 0.390 / 1.691 m 为准。
