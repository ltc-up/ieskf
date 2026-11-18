# AUDIT_ARCHITECTURE_COMPARISON — 源码级对照

对照对象：生产 `lidar_ndt`（CUDA-NDT + 既有 `GicpRegistration`）、Phase AW 的 `gicp_service.cpp`、
本地 `small_gicp fa0cfc98`、本地 `fast_gicp 0e7ec14`、本地 `FAR-LIO ef20510`、
15D 研究 `src/geometry_ieskf`。每一行都给出文件/符号。

---

## 1. 五个系统在"配准→状态"链路上的位置

| 系统 | 配准器 | 状态维度 | 观测进入方式 | R 的真实来源 |
|---|---|---|---|---|
| 生产 `lidar_ndt` (CUDA) | CUDA-NDT Gaussian | 外部 EKF（生产） | 松散位姿 + 6×6 协方差 | `applyCudaMeasurementPolicy`：优先 Native 协方差，否则 `fillFitnessCovariance`（fitness 填对角） |
| `GicpRegistration`（生产实验分支） | small_gicp GICP / VGICP，fast VGICP | 同上 | 同上 | **无**，`out.reliable` 恒 false，`exit_reason` 统一；注释明示 "No production reliability/covariance calibration for experimental backends" |
| Phase AW GICP | small_gicp GICP（JSONL 外进程） | 15D ESKF（Python） | `ExternalPose` 6 行（XYZ + prior-anchored SO3 Log） | 固定工程 R：平移 0.5 m、旋转 1° |
| FAR-LIO | 自研 GICP（point-to-distribution）+ 体素哈希 | 9D EKF（pos/欧拉角/body 速度） | 松散 `nav_msgs/Odometry`，`H` 为状态选择矩阵 | `ConstantCovariance` + `min_cov` floor + 适配衰减 |
| 本轮 G1 | 继承 small_gicp GICPFactor + 鲁棒/关联改造 | 不含状态估计（G1 边界） | 输出诊断，不更新滤波器 | 不产生 R；`information.covariance_status = "NOT_A_COVARIANCE"` |

**结论**：本研究中没有任何一个系统把 GICP Hessian 逆当作 EKF 的 R。生产 CUDA 路径用的是
NDT **自身**的 Native 协方差（`ndt_gpu_matcher` 内部），不是 GICP Hessian。

---

## 2. 点协方差：四种实现的实际差异

| 实现 | 文件 / 符号 | kNN | 是否含自身 | 特征值处理 | 邻居不足时 |
|---|---|---:|---|---|---|
| small_gicp | `util/normal_estimation.hpp: estimate_local_features<CovarianceSetter>` | 20（`registration_helper` 里硬编码 10） | **是**（树含自身，距离 0） | 丢弃实测谱，替换为 `(1e-3, 1, 1)`，特征向量保留（`CovarianceSetter::set`） | `set_invalid` → **各向同性 Identity**（最大错误置信度） |
| fast_gicp (CPU) | `gicp/impl/fast_gicp_impl.hpp: calculate_covariances` | 20 | 是 | `RegularizationMethod::PLANE` → `(1,1,1e-3)`；另有 `MIN_EIG` / `NORMALIZED_MIN_EIG` / `FROBENIUS` / `NONE` | 未初始化列 + 除 k 的潜在缺陷（邻居矩阵 4×k 仅填 k_indices.size() 列） |
| FAR-LIO | `map_handler/map_handler_base.hpp: set_normal_covariance` | `NUM_NEIGHBORS = 10` | 体素邻域内 | `map.cov_regularization`：`SVD`→`(1e-3,1,1)`；`FROBENIUS`→`C=(cov+1e-3 I)` 再归一化；`MIN_EIGENVALUE`→`max(1e-3, λ_i)`；配置实际用 **FROBENIUS** | `cov = 0`（不是 Identity） |
| 本轮 G1 | `local_covariance.cpp: computeFeature` | 可配置，默认 20 | 是（与上游一致） | `SMALL_GICP_UNIT` 精确复刻上游；`SPECTRUM_FLOOR`：`λ_i ← max(λ_i, ρ·λ_max)`，保留空间尺度 | 与上游一致（Identity）并计数 `insufficient_neighbours` |

**这是最重要的一条审计结论**：small_gicp 的协方差**不是**测量到的点噪声协方差，而是
"平面几何度量"。三个库的默认 `(1e-3,1,1)` 都把每个邻域压缩成**单位尺度**的圆盘，
因此 `RCR = C_t + T C_s T^T` 中两侧尺度都被归一化，Mahalanobis 只保留**方向**信息。
FAR-LIO 的 `FROBENIUS` 模式是唯一保留尺度的现成替代。

---

## 3. 对应与关联

| 项 | small_gicp | fast_gicp | FAR-LIO | 生产 CUDA-NDT | 本轮 G1 |
|---|---|---|---|---|---|
| 结构 | 自建 KdTree | PCL KdTree / 自建体素图 | **仅体素哈希** | GPU 体素高斯 | KdTree（目标可 ROI 裁剪） |
| 邻域模式 | 由 rejector 决定 | DIRECT1/7/27 | 固定 27 | DIRECT7（冻结）+ GLL | 不适用 |
| 距离门限 | `DistanceRejector.max_dist_sq`（平方，m²），**普遍生效** | `FastGICP`/`ST` 生效；`FastVGICP`/`VGICPCuda`/`NDT`/`MP` **不生效** | `3σ`，σ 由 `AdaptiveThreshold` 从初值-解偏差估计 | Mahalanobis 门限 | 双侧距离门 + 法向一致性 + 互惠检查 + ROI |
| 互惠/一对一 | 无 | `FastGICPSingleThread` 有第二近邻剪枝 | 无 | 无 | 可选互惠检查 |
| 跨迭代缓存 | 无（每次重新搜索） | `correspondences_` 缓存索引 | 无 | 无 | 无 |

**AW 已披露的缺陷**：`GICP_INTEGRATION.md` 明确写 "Global nearest-neighbor search followed by ROI
rejection does not retry the next in-ROI neighbor"。本轮 `crop_target_to_roi=true` 正是修这一条：
把树建在 ROI 子集上，使拒绝后的候选不会因"最近邻在 ROI 外"而整体丢失。

---

## 4. 鲁棒核与优化

| 项 | small_gicp | fast_gicp | FAR-LIO | 本轮 G1 |
|---|---|---|---|---|
| 核 | `RobustFactor<Huber|Cauchy>` **存在但 helper 不用**，且 `c` 作用于 `sqrt(e)`，单位与 `max_dist_sq`(m²) 不一致 | 无 | **Cauchy**，`k = σ/3`，**在因子内部**（`JTJ = Jᵀ w P J`） | `RobustGICPFactor`：`w(r)`，r 为**米制**欧氏残差；Huber / Geman-McClure，可在因子内关断 |
| 优化器 | LM（默认）/ GN | LM（`lm_init_lambda_factor=1e-9`） | GN（`max_iter 500`, `5e-3`, `200 ms`） | LM（继承）, 50 iter, 1 mm / 0.1° |
| 扰动侧 | **右** `T·exp(δ)` | **左** `exp(δ)·T` | 左 `J_r=[I|−p̂]` | 右（继承 small_gicp） |
| Hessian 语义 | 冻结 M 的 GN Hessian，`∂M/∂T` 被丢弃；且 `result.H` 在**步进前**位姿 | `final_hessian_` 在 LM 接受后 | 无导出 | 在**最终位姿**重新线性化（`registration_diagnostics.cpp`） |
| 多分辨率 | **无**（grep 零命中） | 无 | 无（仅降采样） | 可选 coarse(1.5 m)→fine，默认关 |

---

## 5. 健康性 / 退化判断

| 能力 | small_gicp | FAR-LIO | 生产 | 本轮 G1 |
|---|---|---|---|---|
| 特征值谱 / 条件数 | 不提供（需调用方自己算 `result.H`） | **NOT FOUND** | CUDA Native 协方差内含 | `analyseInformation`：6×6 谱、条件数、最弱平移方向、`rank_deficient` |
| inlier 比例 | 只有 `num_inliers`（末次线性化计数） | 无（只有 `converged/duration/num_iter`） | fitness / transform_probability | `pairs`、`candidate_pairs`、逐门限拒绝计数、inlier 比例 |
| 残差分布 | 只有标量 `error`；逐点残差**不保留** | 仅 LM 的 `debug_.conditional["error"]` | fitness | 欧氏 + Mahalanobis 的 mean/median/p95/max，及 IRLS 权重统计 |
| 空间覆盖 | 无 | 无 | 无 | 12 扇区直方图、占用扇区数、主导扇区比例、熵、近场比例 |
| 退化方向 | 无 | 无 | 无 | 平移块 3×3 最小特征向量 + 比值 |

FAR-LIO 的 9D EKF 侧确实有状态机（`submodules/state_machine.hpp`，OK/WARN/ERROR/STALE 锁存）
和 Mahalanobis **截断**（`helper/outlier_rejection.hpp:187`，不是拒绝），
但**配准层**没有任何退化检测。所以"FAR-LIO 整体性能"不能归功于 GICP 数学。

---

## 6. Deskew 与状态反馈（决定 G2 形态）

| 项 | FAR-LIO | 15D 研究（AR/AS/AW） |
|---|---|---|
| 去畸变来源 | **TF 位姿历史多项式**（10 ms 网格、200 ms 窗、1 阶 se(3)、Horner 求值），**完全不用 IMU** | CUDA Deskew 服务，输入 `[dt, quat xyzw, trans]` 20–24 个快照（即 `_poses.bin`），由 15D 滤波器的**后验**轨迹给出 |
| 反馈方向 | EKF 输出 TF → 里程计去畸变 | 15D 后验 → Deskew → Source → Seed → 匹配 → 15D |
| 延迟处理 | 前向外推 `DelayCompensation`；**不支持乱序/回滚** | `DelayedLocalReplay` 500 ms checkpoint + 重放，支持乱序/迟到/重复 |
| 时间基准 | `TS = 0.01 s` 固定 100 Hz | 事件驱动，`(header_ns, kind_rank, message_id)` 排序，int64 ns |

**因此**：FAR-LIO 的"几何前端"经验可借，但它的**状态融合不能照搬**——9D/45 维测量与
15D 在结构上不同，且它没有延迟回滚。G2 必须沿用 15D 的 `DelayedLocalReplay` 语义。

---

## 7. Phase AW 与本轮的真实关系

读 `ieskf-aw/experiments/phase_aw_native_gicp_fusion/gicp_service.cpp` 与 `GICP_INTEGRATION.md`：

- 相同点：同一上游 `fa0cfc98`、同一 `GICPFactor`、同一 LM、同一右扰动、同一 50 iter / 1 mm / 0.1° / 4 m / kNN20 / TBB-4。
- 不同点（本轮 G0 与 AW 的**实现**差异，不是配置差异）：
  1. AW 在 **stdin JSONL 服务**里跑，Target 树与协方差只建一次并跨帧常驻；
     本轮在进程内 `GlobalMapManager` 缓存，语义相同。
  2. AW 的 ROI 用**自定义 rejector**在全局 NN 之后剔除；本轮 G0 复刻同一语义，G1 才改为裁树。
  3. AW 的 `curvature_probe` 用**冻结因子**做 ±步长探测；本轮改为在最终位姿**重新线性化**，
     因此本轮报告的 H/残差属于返回的那个位姿，不是上一次迭代的位姿。
- 数值等价性由本轮实测给出（见 `G1_VALIDATION_RESULTS.md` §2）：1,468 帧中位平移差 6e-6 m、
  p95 8.6e-3 m、最大 0.28 m；中位旋转差 7e-6°；inlier 计数差中位 0。
  差异集中在少数 LM 收敛边界帧，属于优化器路径差异，不是配置不同。

---

## 8. 未启用 / 不可用的能力（诚实清单）

- small_gicp：**无**多分辨率、**无**退化检测、**无**逐点残差保留、**无**增量体素图在 CPU 侧的
  "点云更新"接口（`IncrementalVoxelMap::insert` 有，但 voxel cov 是"已正则化的点协方差之和"，
  不是空间协方差）。
- fast_gicp：`FastVGICP`/`VGICPCuda` 忽略距离门限；VGICP 的 `w=sqrt(num_points)` 权重恒开；
  `ADDITIVE_WEIGHTED` 未实现。
- FAR-LIO：无 IMU 去畸变、无 kd-tree、无退化检测、无多分辨率、无回滚；
  `submodules/TAM__common` 在本机为空，`DelayCompensation` 内部实现 **NOT FOUND**。
- 生产：`GicpRegistration` 不产生协方差/可靠性；`ndt_gpu_matcher` 的 Native 协方差是
  "averaged best-voxel inlier GN + 谱 floor"，**不是**精确 score Hessian 逆。
