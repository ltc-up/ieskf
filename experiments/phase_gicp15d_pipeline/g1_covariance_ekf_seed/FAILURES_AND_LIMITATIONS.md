# FAILURES_AND_LIMITATIONS

## F1 — FROBENIUS 是负结果，不是"更好的协方差"

`G1_FAR_FROBENIUS` 在 8 个可判定帧上 p50 0.373 → 1.691 m、mean 0.880 → 2.561 m，
优于 NDT 的帧数 5 → 3，并且使 4/18 帧由收敛变为不收敛。
**FAR-LIO 的公开配置选择（FROBENIUS）在本项目的地图/参数下不成立。**
没有通过调 λ 或 ρ 去挽回；λ 固定为源码字面 `1e-3`。

## F2 — G0 的优势有明确边界，且存在反例

| 反例 | 事实 |
|---|---|
| C4_18 | Seed 仅偏 0.446 m、NDT 0.084 m，G0 却给出 1.083 m |
| C4_20 | Seed 1.454 m，G0 0.373 m；NDT 0.071 m 更好 |
| C4_19 | G0 0.120 m，NDT 0.111 m，两者相当 |
| **C4_25** | **NDT 0.226 m，G0 4.388 m —— G0 进入错误盆地** |

即 G0 不是"总是更好"，而是"在 NDT 失效的 3 帧上更好，在 NDT 正常时可能更差，
并且在失败段第 4 帧同样失败"。

## F3 — 上一轮 G1-R 的口径错误（已更正）

三处，均已在本轮改为真实输入：

1. 代理取错：用了 phase_e4 narrow 的 NDT final，而非 `WALL_frames.csv` 的
   `ndt_x_good/ndt_y_good`；
2. Target 被重建（98,214/98,216 点）而实际上历史 `target.bin`（98,213 点，
   sha `cfff1a6c…`）一直存在；
3. Source 用了 phase_e4 的序列化（sha `50ae6316…`），而真正历史 Source 是
   `phase_bg_far_lio/c4/frozen/22/source.bin`（sha `964d20da…`）。

上一轮 G1-R 的 0.331 / 0.969 m 不应再被引用。本轮更正为 0.390 / 1.691 m。
**G1-R 的定性结论（G0 脱离 NDT 错误分支）在更正后仍然成立。**

## F4 — Seed 来源只能标 `SEED_PROVENANCE_PARTIAL`

- 可确认：Seed 是匹配请求中被提交的 `seed_common`，与 Source/Target 同帧同坐标系，
  且随帧单调漂移 0.45 → 2.60 m。
- 不可确认：是否逐帧来自当帧 EKF 预测（无逐帧 P / IMU 记录）。
  KG-S2 对同源帧标注 `conditional offline5scan graph ... zeroBias/defaultP`。

## F5 — 样本量与时段

- 可判定帧 **8 个**，失败段 **3 帧**（约 0.15 s）。不能外推到整圈。
- 26 个不同 header 中 18 个**无参考**，只能报几何量，标 `AMBIGUOUS`。

## F6 — 参考是工程代理，不是真值

`ndt_x_good/ndt_y_good` 是历史 0105 稳定 NDT 轨迹，`WALL_frames.csv` 自身
`fitness_score = inf`、`match_score = 0.148`，且该行 `reject = True`。
所有"到代理的距离"只能在同一代理口径内比较。

## F7 — Target 点数不一致

`WALL_frames.csv` 该行 `target_points = 98223`，冻结 `target.bin` 为 98213 点
（`map_version 1042` 一致）。同一 ROI 的不同快照，差 10 点。
本轮使用冻结 `target.bin`，并保留该差异。

## F8 — 未执行的部分

- 未做 G2 C++ 15D 实现、未做连续闭环、未做真实时钟。
- 未测 5925/5931（上轮已判定 `NOT_COVERED`，本轮未改变）。
- 未做参数扫描（ρ、λ、kNN、距离门、最大迭代）。
- 未做线程/归约后端归因（上轮 19 帧分歧仍为 `PARTIAL`）。
- 未在 C4 系列上使用多初值探针（本轮只做单 Seed 主实验）。

## F9 — 一项方法学提醒

FROBENIUS 把条件数从 2.4e4 降到 7.5e3，却使精度显著变差。
**"Hessian 条件数更好"不能作为前端改进的证据**——这条在本轮得到了直接反例。
