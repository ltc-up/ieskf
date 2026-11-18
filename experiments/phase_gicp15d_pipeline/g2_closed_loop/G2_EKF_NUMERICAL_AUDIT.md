# G2_EKF_NUMERICAL_AUDIT

对象：`src/geometry_ieskf`（Python 15D ESKF）@ 本分支。本文件是审计结论，不是重写。

## 1. 状态与坐标

15 维误差状态顺序 `[dp(0:3), dv(3:6), dθ(6:9), dbg(9:12), dba(12:15)]`，
`State` 为 `p, v, R, bg, ba, t`（`state.py:3-10`）。姿态误差**右乘**：
`inject: R ← R·Exp(dθ)`；协方差 reset 用 `chart_map(d) = diag(I,I,Jr(dθ),I,I)`。
`p` 是 **IMU 原点在 map 系**，base_link 由 `p − R·lever` 得到。
GICP 观测与状态**同坐标系**（map ← base_link），已经在 `external_pose.py` 中一致表达。

## 2. IMU 传播

`inertial.py:model`：ZOH 偏置、`Eh=Exp(ω dt/2)` 求加速度、`E=Exp(ω dt)` 求姿态、
`p += v dt + ½a dt²`、`v += a dt`，每子步 `Rotation.from_matrix(y.R).as_matrix()` 重正交。
`F`、`G` 解析给出；`Q = diag(repeat([σg²/dt, σa²/dt, σbg²·dt, σba²·dt], 3))`。
`P_new = F P Fᵀ + G Q Gᵀ`，随后 `(P+Pᵀ)/2`。
`dt ≤ 0` 或 `dt > 0.025` 触发拒绝（`max_imu_interval_s`）。
**注意**：`Abg` 用 `Jr(ω dt/2)`，而 `F[6:9,9:12]` 与 `G[6:9,:3]` 用 `Jr(ω dt)` —— 两个参数不同，
C++ 移植必须保留。

## 3. Wheel

单行纵向（body-x）速度观测，`H[0,3:6]=Rᵀ[0]`、`H[0,6:9]=skew(vb)[0]`、`H[0,9:12]=−skew(lever)[0]`；
方差 `variance + 0.5²`；门控 `INVALID / STALE / ZERO_CONFLICT / SLIP_OR_MISMATCH(5 m/s)`。
**`age` 从未被调用方传入**，因此 `STALE` 分支实际不可达 —— 是一个死门（`wheel.py:11-14`）。
本轮未修改（无证据表明它造成故障），但记录在案。

## 4. LiDAR 观测

`ExternalPose`（`external_pose.py:28-46`）：残差 = base 平移 + `Log(R_priorᵀ R_meas)`，
`H[:3,:3]=I`、`H[:3,6:9]=R·skew(lever)`、`H[3:,6:9]=Jr(rv)⁻¹`（线性化点处 = I）。
闭合环中 R 由 `run_loop.py` 覆写为固定 `diag(0.5²,0.5²,0.5²,(π/180)²×3)`。
**没有**任何路径用 GICP fitness 或 Hessian 作为 R。

## 5. 协方差更新

Joseph 形式：`A=I−KH`、`Pc = A P Aᵀ + K R Kᵀ`、`Pc = B Pc Bᵀ`、再对称化，
`valid_prior()` 校验对称性与 Cholesky 正定。reset **只做一次**。
`K` 裁剪、协方差 floor、P 膨胀：**NOT FOUND**（有意如此）。

## 6. 时间与延迟观测

`DelayedLocalReplay`：500 ms horizon，`snapshots` 为每个事件后的 `(x,P,imu)`，
排序键 `(header_ns, kind_rank, message_id)`（imu 0 / wheel 1 / clock 2 / ndt 3）。
拒绝码：`LATE`、`IMU_GAP`、`DUPLICATE`、`OUTSIDE_CHECKPOINT_HISTORY`、
`INPUT_OUTSIDE_FORWARD_HORIZON`、`BUFFER_CAPACITY`、`MISSING_CORRELATION_GROUP`、
`DUPLICATE_INFORMATION_GROUP`。
迟到的 LiDAR 观测：回滚到 `ref` 的快照 → 应用观测 → 确定性重放后缀。

**Seed 与 Deskew 的来源（已核实于 `run_closed_loop.py`）**：
- Seed = `pose(checkpoint(ref).prior, lever)`，即 `ref` 时刻的**预测**（已含此前所有后验）；
- Deskew 位姿历史 = `[s[0] for s in replay.snapshots if spanlo−20ms ≤ s.t ≤ ref]`，
  即**后验轨迹**（每个事件之后的快照）。
两者都不是"当前 LiDAR 更新之后"的状态 —— 闭环在这一点上是正确的。

## 7. 数值风险清单（C++ 移植合同）

1. `Jr` 小角分支阈值 `a < 1e-5` 及 Taylor 系数 `(.5−a²/24)`、`(1/6−a²/120)`。
2. `Abg` 用 `Jr(ωdt/2)`，姿态 bias 列用 `Jr(ωdt)` —— 不可统一。
3. 每子步 matrix→quat→matrix 重正交。
4. 事件驱动子步长（不是固定 100 Hz）。
5. `t += round(dt*1e9)`，int64 ns，Python `round` 是 banker's rounding。
6. Joseph 算术顺序与 `var·K Kᵀ`（1×1 R 的秩 1 形式）。
7. `cho_solve` vs `P Hᵀ/S`（位姿用前者，wheel 用后者）。
8. `Pc = B Pc Bᵀ` 中 `B` 非正交（`Jr`），对称化在之后。

## 8. 本轮可执行的健全性检查（结果）

| 检查 | 结果 |
|---|---|
| 帧 0 与 AW 归档的 `p`、`P` 对角 | **完全相同**（dp=0，dP=0） |
| 1468 帧 P 的有限性 | 全部有限（由 `valid_prior` 强制） |
| 收敛/接纳一致性 vs AW | 1468/1468 |
| 回放确定性 | **失败**，见 `G2_FAILURE_ANALYSIS.md` F1（属驱动器，非滤波器） |

**结论**：15D 数学本身未发现缺陷；发现的问题是**死门（wheel STALE 不可达）**
与**驱动器时间依赖**，两者性质不同，都不构成本轮精度结论的依据。
