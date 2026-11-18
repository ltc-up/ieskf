# G0_NUMERICAL_DISCREPANCY

**NEW_EXECUTION**（25 帧定向复跑 + 代码缺陷修复）。不重跑 1,468 帧。

## 1. 多初值探针 Cost 缺失：真实缺陷，已修复

### 缺陷确认

`src/gicp_frontend.cpp` 的 `GicpFrontend::probe()` 在 `runPass()` 之后**只**填了
`final / converged / candidate_valid / iterations / inliers / seed_*_correction`，
**从未**写 `out.residuals` 或 `out.information`。而 `align()` 里做

```cpp
pr.cost = one.information.cost;
```

`RegistrationOutput::information.cost` 的默认值是 `kNaN`，所以每个探针的 `cost` 恒为 NaN。
缺陷成立，已由代码复核 + 修复前实测（NaN）确认。

### 修复

在 `probe()` 末尾，对**该探针自己的最终位姿**调用与主结果完全相同的
`computeDiagnostics(...)`：同一 `GicpRejector`、同一 `RobustGICPFactor::Setting`、
同一线程数、同一 Source/Target。这样探针成本与主结果属于**同一目标函数、同一残差定义**，
才允许横向比较。若重算失败，`exit_reason` 置为 `PROBE_COST_UNAVAILABLE`，
不允许静默 NaN。

### 新增测试（`tests/run_tests.cpp::testProbeCostContract`）

断言四件事：

1. 主结果 `information.cost` 有限；
2. 每个探针 `cost` 有限，或该探针 `correspondences == 0`（有明确失败原因）；
3. **零偏移探针的 cost 与主结果 cost 相等**（相对 1e-9）——这是"成本可比较"的可执行证明；
4. 零偏移探针确实存在。

修复后测试总数 **41 → 48**，全部通过（`48 checks, 0 failures`）。
主匹配结果不因该诊断修复而改变：`probe()` 只在 `align()` 的探针分支被调用，
主 `align()` 路径的 `runPass` 与诊断代码未被触碰。

## 2. R1 的 19 帧 >10 cm：线程数假设被证伪

### 方法

从上轮 `aw_equiv_g0.jsonl` 与 AW `matcher_ab_fixed/matches.jsonl` 中取出
平移差 >0.10 m 的全部 **19 帧**，另加 6 帧 <1 mm 的正常对照，共 25 帧。
在 `--threads 1` 与 `--threads 8` 下各复跑一次 `G0_AW_EQUIV`。

### 结果

| 指标 | threads=8 vs AW | threads=1 vs AW |
|---|---:|---:|
| 平均 \|ΔXY\| | 0.13181 m | 0.13181 m |
| 最大 \|ΔXY\| | 0.27999 m | 0.27999 m |
| threads=8 vs threads=1 最大差 | — | **7.1 × 10⁻¹⁵ m** |

6 帧对照的 `dAW` 全部为 **0.0000**，`iterations` 与 `inliers` 与 AW 逐项相同。

### 结论

**"OMP 归约顺序导致 19 帧分歧" 这一假设被证伪。**
单线程与 8 线程结果一致到 1e-14 m 量级，却仍与 AW 相差 0.28 m。
线程数不是根因。

### 仍然成立的候选（未分离，不声称）

1. **归约后端不同**：Phase AW 的 `gicp_service.cpp` 使用
   `small_gicp::ParallelReductionTBB`（TBB arena 4）；本前端使用
   `ParallelReductionOMP`。TBB 的 `blocked_range` 划分与 OMP `schedule(guided,8)`
   不同 → 求和顺序不同 → 在病态帧上使 LM 路径分叉。这是目前**最符合证据**的解释。
2. **LM 步接受/拒绝路径**：两边的 `max_inner_iterations / init_lambda / lambda_factor`
   均为上游默认值，但 H/b 的末位差异可改变 `rho` 的符号。
3. 已排除：Source 身份（逐帧 `source_sha256` 相同）、Target 身份（同为 `0895ad26…`）、
   迭代上限、收敛阈值、最大对应距离、协方差 kNN、协方差模式（均为 UNIT）。

### 为什么没有继续分离

切换归约后端需要修改 `Registration` 模板实例化并引入 TBB 依赖，
属于"改变求解器实现"而非"归因诊断"。在 G1 主结论不依赖这 19 帧的前提下，
本轮按任务书要求**记录并隔离**，不扩大改动面。

### 对 C4_22 主实验的影响评估

无。C4_22 的两条结论（G0 恢复到 0.331 m；G1_SPECTRUM 变差到 0.969 m）差距 0.64 m，
比 0.28 m 的最大分歧大一个量级；且 C4_22 的 Target 是 98k 点局部云，
与 R1 的 908k 全图 Target 不同。因此该分歧不改变 C4_22 的判读。
