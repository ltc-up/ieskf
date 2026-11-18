# G2-R：回放确定性与重新评价

基线提交 `aa90e19`。分支 `dsh/gicp15d-pipeline`。生产仓库未改，GICP 算法与 Q/R 未改。

## 1. 两处非确定性，而不是一处

### (a) 驱动器的时钟依赖（已修复）

冻结的 `run_closed_loop.py` 在两次推进中用实测墙上时间：

```python
latency=(perf_counter_ns()-start)/1e6;  advance(job_now+round(latency*1e6))
...
advance(job_now+round(perf_counter_ns()-start))
```

修复：新增 `LOGICAL_LATENCY_NS = 0`，两处 `advance` 改用该**声明常量**；
`perf_counter_ns()` 仍保留，但只用于 `wall` 统计，不进入逻辑时钟。
**逐行 diff 只有 2 行被替换 + 6 行常量声明**（见 §6），原始文件字节未动。

### (b) 匹配器的 OMP 归约顺序（新发现，必须一并修复）

只修时钟后重跑 3 次（200 帧，4 线程）：`states.jsonl` 仍不同。
逐帧定位到**第 0 帧**：`seed`、`source_sha256`、`complete` 全部相同，只有 `ndt.final` 不同。

根因：`small_gicp::ParallelReductionOMP` 使用 `schedule(guided,8)`，
**分块在线程间动态分配**，因此每个线程累加哪些点每次不同 → 部分和求和顺序不同 →
末位不同 → LM 路径分叉。这是与时钟无关的第二个独立缺陷。

处理：闭环匹配器以 `--threads 1` 运行（单线程归约顺序固定）。
**这不是"性能调优"，是确定性前提**；性能另用 4 线程单独测量（§4）。

## 2. 确定性验收（3 次重复，200 帧）

| 检查 | 结果 |
|---|---|
| `states.jsonl`（后验轨迹 + 15×15 P）sha256 | **三次完全相同** |
| 每帧 `seed` / `prior`(p,R,P) / `ndt.final` / `update` | **逐位相同** |
| 每帧 `complete`（逻辑完成时刻） | 相同 |
| 3×200 帧逐帧对比，排除计时字段后的差异字段 | **NONE** |
| 仅存的差异 | `align_wall_ms` 等**性能字段**（按要求不入逻辑时钟） |

对照：修复前同一二进制两跑，后验 p50 0.081 m / max 0.53 m 不同；现在为 0。

## 3. 重新运行的三条闭环（同一确定性驱动，全圈 1468 帧）

| 配置 | 接纳 | 丢弃 | 参考 RMSE | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|---:|
| **G0 + 15D** | 1401 | 67 | **2.466** | 2.425 | 3.690 | 4.270 | **4.916** |
| NDT legacy + 15D | 1467 | 1 | 2.622 | 2.536 | 3.906 | 4.476 | 5.134 |
| NDT fixed-R + 15D | 1467 | 1 | 2.506 | 2.469 | 3.755 | 4.306 | 4.908 |

同一参考 `G_0.075`、同一 1197 帧有效掩码、同一杆臂、同一对齐——
**三者互比有效；绝对值仍不可引用**（见 §5）。

判读：

1. 在确定性驱动下，**G0 比 NDT legacy 好 0.156 m RMSE（−5.9%）、
   比 NDT fixed-R 好 0.040 m（−1.6%）**，且 `max` 也更小。
2. 差异现在**大于噪声**（噪声为 0），但**小于参考自身的不确定性**，因此
   只能作为"条件性几何收益"，不能作为精度定论。
3. 代价：G0 多丢弃 66 帧（4.5% vs 0.07%）。它在更少的观测下取得更好轨迹，
   说明丢弃的帧里包含错误匹配——这与 G1-C 的结论一致。
4. 值得注意：本轮绝对值（2.5–2.6 m）与 G2 报告的 3.0–3.4 m 明显不同，
   **说明时钟模型本身对轨迹有 ~0.8 m 量级的影响**，此前所有基于该驱动的
   绝对精度数字都应视为时钟伪迹。

## 4. 实时性（诚实代价）

| 配置 | matcher align p50 | p95 | p99 | max | 帧 wall p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| G0 1 线程（确定性） | 16.29 | 38.75 | 64.05 | 79.59 | 28.07 | **51.67** | 79.56 | 116.25 |
| G0 4 线程（G2 实测） | 5.72 | 12.53 | 20.32 | 32.78 | 20.12 | 30.36 | 47.05 | 117.29 |

**确定性版本 p95 51.67 ms > 50 ms，不满足 20 Hz**；4 线程版本满足但不确定。
这是本轮最重要的工程结论：**确定性与实时性当前互斥，因为归约调度是动态的。**

## 5. 为什么没用 `evaluate_reference.py`

该脚本需要 8 个输入：`--input --run --gnss --center --left --right --raceline --private`。
实测缺失：

- 正确时间约定的 `gnss_baselink_truth_delay75_full_validated.csv` —— 磁盘上不存在
  （只有另一 capture/map frame 的 `gnss_truth_latest_gll_validated.csv`）；
- `center/left/right/raceline` 四个 CSV —— `data/phase_as_accuracy_quality/` 下均不存在。

因此**没有签核管线的评价**。本轮改用 AW 归档 `evaluation.npz` 里仅存的
`G_0.075` 与 `historical_valid_0.075`，三条闭环用**完全相同**的对齐代码打分，
保证互比公平，但绝对尺度（2.5 m 量级 vs AW 报告的 0.596 m）说明我的对齐实现
与签核管线不等价。**绝对 RMSE 不可引用。**

## 6. 代码改动（可审计）

```
run_closed_loop_det.py  vs  experiments/phase_ar_loop44_map0105/run_closed_loop.py
  15a16,21  + LOGICAL_LATENCY_NS = 0（含注释）
  78c84     - latency=...;advance(job_now+round(latency*1e6))
            + latency=...;advance(job_now+LOGICAL_LATENCY_NS)
  83c89     - advance(job_now+round((perf_counter_ns()-start)))
            + advance(job_now+LOGICAL_LATENCY_NS)
```

`run_loop_det.py` 仅把 `source=` 指向上述副本的绝对路径。
`matcher_g0_t1.sh` 是 `exec gicp15d_g0_service --threads 1` 的包装
（驱动只接受单个可执行路径，不能带参数）。

## 7. 结论与下一步

- 回放**已确定**：同一输入三次运行，状态与 P 逐位相同。
- 在确定性条件下，**G0 相对 NDT 有小而一致的几何收益（−0.16 m RMSE）**，
  代价是 4.5% 的观测丢弃与**不再满足 20 Hz p95**。
- 单一阻塞：**归约调度**。把 `ParallelReductionOMP` 换成
  `schedule(static)` 的确定性归约（或自带 `DeterministicReductionOMP`），
  即可同时拿到确定性与 4 线程性能，那时才值得进入 FAR-LIO 机制移植与故障注入。
- 在有签核 GNSS 参考与赛道 CSV 之前，任何绝对精度结论都不成立。
