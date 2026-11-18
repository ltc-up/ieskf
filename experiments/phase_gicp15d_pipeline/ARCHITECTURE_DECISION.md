# ARCHITECTURE_DECISION

决定日期 2026-11-18。依据见 `AUDIT_SOURCE_INVENTORY.md`、`AUDIT_ARCHITECTURE_COMPARISON.md`。

## 1. 一句话决定

保留 CUDA-NDT 为冻结基线；新建**独立 C++ 研究模块** `src/gicp15d/`，以 small_gicp 上游为几何内核，
先做 **G1 可靠几何前端**并在真实冻结数据上验收；15D ESKF 的 C++ 化（G2）与几何健康性融合（G3）
只在其上按阶段推进，**本轮不宣称** G2/G3 通过。

## 2. 复用

| 组件 | 来源 | 复用方式 |
|---|---|---|
| 小 GICP 求解器 | `small_gicp fa0cfc98`（header-only） | 直接 include；`Registration` / `GICPFactor` / LM / 右扰动 / `DistanceRejector` 语义全部保留 |
| 点协方差定义 | 同上 | `CovarianceMode::SMALL_GICP_UNIT` 逐位复刻上游 `CovarianceSetter` |
| 冻结 Target | `map_target.bin`(0105, `0895ad26…`)、`map1013_leaf8.bin`(1013, `2f7c78d7…`) | 只读加载，加载后强制 sha256 校验 |
| 冻结 Source | `F_multilap/<stamp>_A1.bin` | 只读；与 production `preprocessPointCloud` 同合同（crop→0.8 VG→cap5000） |
| 冻结 Seed | `F_multilap/transactions.jsonl` 的 `seed` | 只读；与 Phase AW/AQ2 的 `seed_common` 同一定义 |
| 外参 | `T_body_lidar` / `lever_body_imu_m` | 不改、不重标定 |
| ROI 合同 | cylinder + Z band，`center = seed.xyz`、`r=120`、`z−10…z+30` | 与 `run_closed_loop.py` 完全一致 |
| 参考轨迹 | 仅 278.124–362.423 s 存在（1484/4134 帧） | **不用于在线选解**；本轮 395.7–398.8 窗口无参考，故只报代理指标 |

## 3. 新增（本轮实际写出的 C++）

`src/gicp15d/`：`GicpFrontend`、`GlobalMapManager`、`RegistrationDiagnostics`、
`RobustGICPFactor`、`GicpRejector`、`point_cloud_io`、`local_covariance`、`frontend_main`。
全部 C++17，Eigen + OpenMP + small_gicp，**不依赖 PCL、ROS、CUDA**。
独立 CMake 目标，不进入生产 `colcon`。

## 4. 保持不变的

- 生产仓库 `auto-eagle` 的 `main`/`ndt_test` 与所有默认 YAML：**零修改**（本轮无任何写操作）。
- 生产 `GicpRegistration` / `NdtRegistration` / `measurement_policy`：**零修改**。
- 研究仓库既有 `src/geometry_ieskf` / `src/ndt_global` 的 Python 语义：**零修改**。
- 既有各 worktree、`data/` 下所有冻结输入：**只读**。
- CUDA-NDT：冻结，不再新增 NDT 算法；本轮的 CUDA-NDT 数值只作为**基线对照**出现。

## 5. 明确拒绝

| 拒绝项 | 理由 |
|---|---|
| 用 GICP Hessian 逆当 EKF R | 冻结 M 的 GN Hessian，且 Phase AW 已证明几何/误差相关性不成立（Spearman ≈ +0.049/−0.056）；本轮只把它当 `NOT_A_COVARIANCE` 诊断 |
| 用 GNSS/工程参考在线挑候选 | 违反因果性；且 loop44 参考只覆盖 1484 帧 |
| 用低残差/收敛/H 正定证明位置正确 | 三者都只是几何必要条件 |
| 继续局部 NDT Scan-to-Scan | KG-S3 已正式归档 |
| 首阶段追 20 Hz | 先证明算法有效；本轮只记录耗时 |
| 把 5925/5931 与 loop44 拼接 | 两者在世界坐标上不属于同一 map frame（见 §8） |

## 6. 阶段边界（本轮）

| 阶段 | 本轮状态 |
|---|---|
| G0 只读审计 | **完成**（4 份报告 + 1 份增补） |
| G1 C++ 前端 | **完成**（编译、41 项单测通过、真实数据运行） |
| G1 真实数据验证 | **完成**（1,468 帧 G0 复现 + 206 案例双图对照），结论 **PARTIAL** |
| G2 C++ 15D | **未实施**（仅给出接口与等价性合同） |
| G3 几何健康性融合 | **未实施**（G1 未显示明确收益，按任务书"及时停止"） |
