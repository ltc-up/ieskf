# AUDIT_SOURCE_INVENTORY — GICP-15D 管线研究（Phase GICP15D）

本文件是只读审计的**事实清单**。所有条目均在 2026-11-18 由本机实际 `ls` / `git` / `sha256sum` / 二进制解析得到。
未重新下载任何第三方源码，未修改生产仓库工作区，未写入任何既有实验目录。

标记约定：**VERIFIED** = 本轮直接读取/哈希核对；**INFERRED** = 由已读代码或记录推断，未独立重算；
**NOT FOUND** = 在给定搜索路径下不存在。

---

## 1. 机器与工具链（VERIFIED）

| 项 | 值 |
|---|---|
| host user | `ltc` |
| CPU | 32 线程（`nproc`） |
| RAM | 30 GiB 总，本轮审计时 24 GiB 可用 |
| GPU | NVIDIA GeForce RTX 5070 Ti Laptop GPU，12227 MiB，driver 580.178.04（审计时 1061 MiB 已用） |
| 编译器 | g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0 |
| CMake | 3.28.3 |
| Eigen | 系统 `/usr/include/eigen3`，Eigen 3.4 系列 |
| PCL | `pkg-config pcl_common` = 1.14.0，`libpcl-dev 1.14.0+dfsg-1` |
| TBB | `/usr/include/tbb/task_arena.h` 存在（oneTBB） |
| 根分区 | `/dev/nvme0n1p5` 482G，已用 419G，**可用 39G**（92%） |
| 数据分区 | `/dev/nvme0n1p2` → `/media/ltc/新加卷` 1.4T，已用 1.1T，**可用 327G**（76%） |

**写入策略（已执行）**：大型实验输出只写 `/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/`。
该目录已实测创建成功（`reports/ artifacts/ logs/`），**不**在新加卷不可写时回退系统盘。
源码与报告写入独立 git worktree（见 §2），不写生产仓库。

---

## 2. Git 身份（VERIFIED）

### 2.1 15D 研究仓库 `ltc-up/ieskf`

本机存在 **8 个 worktree**（`git worktree list`），共用同一个 `.git`：

| 路径 | HEAD | 分支 |
|---|---|---|
| `/home/ltc/flyagle/ieskf` | `b143e88` | `research/phase-av-filter-improvement`（主工作区，**有未提交修改**） |
| `/home/ltc/flyagle/ieskf-aw` | `e15e7a2` | `research/phase-aw-native-gicp` |
| `/home/ltc/flyagle/ieskf-ax` | `078c672` | `research/phase-ax-direct-residual` |
| `/home/ltc/flyagle/ieskf-ay` | `364d246` | `research/phase-ay-localization-solution` |
| `/home/ltc/flyagle/ieskf-az` | `f25ec45` | `research/phase-az-velocity-fusion` |
| `/home/ltc/flyagle/ieskf-ba` | `3606769` | `research/phase-ba-multisource-velocity` |
| `/home/ltc/flyagle/ieskf-bb` | `7cba238` | `research/phase-bb-lidar-velocity` |
| `/home/ltc/flyagle/ieskf-bd` | `364d246` | `research/phase-bd-ndt-health-bridge` |

- 主工作区未提交修改：`M tests/test_surface_observation.py`（用户既有改动，**本轮未触碰**）。
- 远端 `origin` = `https://github.com/ltc-up/ieskf.git`，可达（`git ls-remote --heads` 成功）。
- 远端存在 `refs/heads/codex/kg-s3-publication` = `de651abe789369c2f497324b700faa997be19bf8`，
  与任务书给出的 KG-S3 发布提交一致。本地原先**没有**该分支；本轮仅执行
  `git fetch origin codex/kg-s3-publication`（只读获取本项目自己的研究提交，非第三方算法源码）。
- 远端 `main` = `05b73c0`。

### 2.2 本轮研究 worktree（VERIFIED）

```
path   : /home/ltc/flyagle/ieskf-gicp15d
branch : dsh/gicp15d-pipeline   (git worktree add -b ... de651ab)
base   : de651ab  research(KG-S3): publish reviewed dual-track implementation and partial validation
```

从 KG-S3 发布提交分出，因此天然包含 `experiments/phase_kg_s3/`、`experiments/phase_bg_far_lio/`
与 AY/AZ/BA/BB 之后的 `src/geometry_ieskf/`（含 `optical_velocity.py`、`vertical_velocity.py`、
`directional_shadow.py`、`shadow_candidate.py`、`attitude_reference.py`）。
**不使用** `main`，不 `force push`，不改动其它 worktree。

### 2.3 生产 ROS2 仓库 `/home/ltc/flyagle/auto-eagle`（VERIFIED）

| 项 | 值 |
|---|---|
| 当前分支 | `ndt_test`（**不是 main**） |
| HEAD | `aa50e35467f89b4963f882d1f5abaace46d0ce70` `feature: add gicp registration` |
| 上一提交 | `ac22bd2c` `chore(loc_third_party): add small-gicp matcher library` |
| 未跟踪 | `src/a2rl_3rdparty/fusion_vendor/`（用户资产，未触碰） |
| 远端 | `origin` 存在，`main` 等分支不受影响 |

生产仓库**本轮只读**。所有新代码写在本轮研究 worktree 内，通过独立 CMake 目标构建，
不加入生产 `colcon` 默认构建，不修改 `lidar_ndt` 包的任何文件。

---

## 3. 第三方算法源码（VERIFIED，本地复用，未下载）

| 组件 | 本机路径 | 版本身份 | 备注 |
|---|---|---|---|
| small_gicp | `/home/ltc/flyagle/auto-eagle/src/a2rl_3rdparty/small_gicp_vendor/upstream` | commit `fa0cfc983417c044b6036f71dfa448038bb9c820`，archive sha256 `d8db6e76db80e838afe042ac968f1d0b905a2f20ab3e1e51c74c6dc85c4cd3bb`，MIT | `UPSTREAM.json` 记录；`upstream/` 是未修改 `git archive`（仅加 `COLCON_IGNORE`）；无 `.git` |
| fast_gicp | `/home/ltc/flyagle/3rd_party/fast_gicp` | git HEAD `0e7ec1441c99f7be453db2ea216d5de029387417` | 含 CUDA 源码，`BUILD_VGICP_CUDA` 默认 OFF |
| FAR-LIO | `/home/ltc/flyagle/3rd_party/FAR-LIO` | git HEAD `ef20510316ad1f8dbf2e428e4a193f700493eccd` | 含 `lib_cpp/`、`ros_packages/`、`submodules/`（本轮不初始化 submodule） |
| faster-lio | `/home/ltc/flyagle/3rd_party/faster-lio` | 存在（未使用） | 本轮不引入 |
| scancontext / 3d_bbs / autoware_core | `/home/ltc/flyagle/3rd_party/` | 存在（未使用） | 本轮不引入 |

**重要差异声明**：
- `small_gicp` 的 vendor 版本（`fa0cfc98`）是本机唯一可编译的 small_gicp。Phase AW 的
  `GICP_INTEGRATION.md` 记录的也是 `fa0cfc98`，**同一 revision**（VERIFIED，见 §5.3）。
- 但 Phase AW 通过**独立 JSONL 服务**（`experiments/phase_aw_native_gicp_fusion/gicp_service.cpp`）
  调用上游头文件，而生产仓库另有 `GicpRegistration` 适配器（`aa50e354`）。两者共享同一上游
  数学，但缓存/ROI/容量语义不同（见 `AUDIT_ARCHITECTURE_COMPARISON.md` §4）。
- `fast_gicp` 在 `gicp_vgicp_benchmark`（2026-10-15）中已实际编译使用；其 `FastVGICP` 在该版本
  **不应用** 继承的 `max_correspondence_distance`（源码注释 `gicp_registration.cpp:138` 明示）。
- FAR-LIO 的 `submodules/` 未初始化；本轮审计以 `lib_cpp/` 与 `ros_packages/` 源码为准，
  不能声称复现其完整构建。

---

## 4. 地图与坐标系合同（VERIFIED）

### 4.1 全局地图（PCD，只读）

| 文件 | 点数 | sha256 | 说明 |
|---|---:|---|---|
| `/home/ltc/flyagle/data/map_data/map_0105_voxel.pcd` | 67,307,313 | 未重算（1.08 GB） | `FIELDS x y z intensity`，binary |
| `/home/ltc/flyagle/data/map_data/map_1013enu.pcd` | 24,131,546 | `5d165afc16f22aaad785711a37575673b09587f1d934edb5cb6dc85b2f52de8f`（本轮重算） | **非标准 PCL 字段布局**：`FIELDS intensity _ curvature _ Original_cloud_index _ normal_x normal_y normal_z _ x y z _`，`COUNT 1 12 1 12 ...`。必须走 PCL `loadPCDFile` 的正常字段解析，不能按 offset 硬解析 |

### 4.2 冻结 Target 二进制（研究实际输入，只读）

格式：`<u64 n>` + `n × 3 × float32`（xyz，map/ENU 坐标，已按 leaf 0.8 体素质心化）。

| 文件 | 点数 | 字节 | sha256 | 身份 |
|---|---:|---:|---|---|
| `ieskf/data/inputs/map_target.bin` | 908,013 | 10,896,164 | `0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968` | 0105 图 |
| `ieskf/data/phase_ar_loop44_map0105/map0105_target.bin` | 908,013 | 10,896,164 | 同上（**逐字节相同**） | 0105 图 |
| `ieskf/data/phase_ar_loop44_map0105/map0105_target_reproduced.bin` | 908,013 | 10,896,164 | 与上两者 `cmp` = IDENTICAL | 复现性证据 |
| `/home/ltc/flyagle/analyzer/lidar_ndt/audits/ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin` | 997,680 | 11,972,168 | `2f7c78d7aa61d219033a3cb9bc1822204db450e9d3ce0405cd1bca9fdcd197ac` | **1013 图**，与 KG-S3 `environment_and_input_identity.json` 记录一致 |

**结论**：0105 与 1013 两套 Target **都有冻结二进制且 SHA 可核**。这使"同 Source/同 Seed 换地图"
的受控实验在 G1 阶段可以直接做，不需要重新从 PCD 生成 Target。
`map1013_leaf8.bin` 的来源在
`ndt_source_target_resolution_20261017/SOURCE_TARGET_PIPELINE_AUDIT.md` §A2 中被明确记录：
"原PCD `map_1013enu.pcd` … Target.8 SHA `2f7c78d7…`，与历史 actual capture 逐字节一致"，
即它不是本轮新造，而是历史实际 GPU Target 的冻结副本。

### 4.3 Source 合同（生产，VERIFIED 于源码）

`NDTProcess::preprocessPointCloud`（`lidar_ndt/src/ndt_process.cpp:270–310`，由
`SOURCE_TARGET_PIPELINE_AUDIT.md` 逐行核对并附行号）：

1. `CropBox`：base_link 下 X ∈ [−100, 100]、Y ∈ [−35, 35]、Z ∈ [1, 15]；
2. `pcl::VoxelGrid`，`num_segments = 1`，leaf = yaml `leaf_size`（当前 0.8 m）；
3. `RandomSample(cap=5000, seed=42)`（`ndt_process.h:152–155`），未超 cap 时 `std::shuffle` 不触发。

Source 恒在 **base_link**；Target 与 Seed 在 **PCD-map (ENU)**。`ndt_registration.cpp:65–73`
仅逐点复制 XYZ，GPU align 不再降采样。

### 4.4 外参

- `T_body_lidar` 逐雷达，记录于 `bindings_multilap.json` 每帧 `clouds[].T_body_lidar`（VERIFIED，
  例：front 雷达平移 `(2.308, 0, …)`）。
- `T_imu_bosch ← base_link` 的杆臂由 `ndt_global.validation.static_transform(data['transforms'],
  'base_link','imu_bosch')` 取得，AR/AS 回放中直接使用。
- **本轮不重新标定外参**，只复用这些已冻结值。

---

## 5. 可用真实冻结数据（VERIFIED）

### 5.1 原始 bag（只读，不重放）

| 文件 | 大小 | sha256 | 说明 |
|---|---:|---|---|
| `/home/ltc/flyagle/data/rosbag/loop44/loop44_0.db3` | 70,760,001,536 | `0be29d090557accc27581007b45660cb5b0063dbcbdd3f3b798e33ff64c1db14`（**沿用历史记录，本轮未重算 66 GB**） | `recorder_first_ns = 1763205419863639180`。**不含**融合话题 |

该 bag 提供时间基准：`offset(t) = (stamp_ns − 1763205419863639180) × 1e−9`。

### 5.2 冻结 Source/Target/Seed 语料

**(A) loop44 双图语料（G1 主用）**

`/home/ltc/flyagle/ieskf/data/phase_as_accuracy_quality/F_multilap/`（VERIFIED）

- 4,134 帧，每帧 4 个文件：
  - `<stamp>_A1.bin`：**matcher 实际输入 Source**，格式 `<u64 n>` + `n×3 float32`，n≈2,700–2,900；
  - `<stamp>_native.xyzi`、`<stamp>_common.xyzi`：中间表示；
  - `<stamp>_poses.bin`：**Deskew 位姿历史**，`float64`，`(N,8)` 行 = `[dt, qx,qy,qz,qw, tx,ty,tz]`，
    20 个快照覆盖约 62.5 ms，**最后一行的 `dt = 0`**，即 `t = ref` 时刻的滤波先验位姿；
  - `deskew_meta_stream.jsonl`（每帧 `raw_points / cropped_points / voxel_points / source_points`）。
- 时间覆盖：offset **273.556 s – 480.206 s**（由 `transactions.jsonl` 的 `ref` 与 §5.1 基准算出）。
- **含 395.7–397.1 s 窗口**：32 帧，`index` 2441–2472（VERIFIED）。

`transactions.jsonl`（4,134 行）每帧完整记录（VERIFIED，逐键读取）：

```
index, ref, available, complete, source_group, source_sha256
seed                       # 4x4，map<-base_link，matcher 的实际 seed_common
prior.p, prior.R, prior.P  # 完整 15D 先验状态与 15x15 P
ndt.{final, fitness, fitness_complete, converged, candidate_valid, exit, iterations,
     seed_float, kappa_at_float_pose, inliers_at_float_pose, neighbor_points_at_float_pose,
     valid_pairs, candidate_pairs, optimizer_score, score_gradient, score_hessian,
     native_internal, parameters_internal, align_internal_ms, align_wall_ms, source_n}
update.{accepted, reason, R, delta, delta_proposed, K, innovation, conditional_NIS,
        components, covariance_calibrated, covariance_updates}
```

`states.jsonl`（4,134 行）为后验状态序列。
**这是 G1/G2 最完整的一个真实语料**：同一帧同时具备 Source、Seed、完整 15D 先验 P、
以及 CUDA-NDT 基线的姿态/Hessian/接受结果，并且覆盖研究要看的首次故障窗口。

**(B) Phase AW 语料（G0 等价性校验用）**

`/home/ltc/flyagle/ieskf/data/inputs/`：

- `legacy_sources/`：**284** 个 `<stamp>.bin`，covering offset 约 423.4–438.0 s；
- `legacy_index.jsonl`：1,398 条，字段 `availability_ns / stamp_ns / offset / path / raw_points / source_points`；
- `source_binding.json`：1,398 个 header → source sha256；
- `map_target.bin`：见 §4.2。

**关键缺口（VERIFIED）**：`legacy_index.jsonl` 中 1,398 条 `path` 全部指向
`.../src/a2rl_localization/lidar_ndt/audit/native_covariance_20261024/.../captured_sources/`，
该目录在生产仓库中 **已不存在**（`ls` 失败）。真实副本位于
`/home/ltc/flyagle/analyzer/lidar_ndt/audits/native_covariance_20261024/phase_f1_f2_20261025/results/captured_sources/`，
共 **1,399** 项（1,398 个 `.bin` + `index.jsonl`，55 MB），首条 `1763205769899987456.bin` 存在。
**因此 1,398 帧 legacy 源可通过路径重绑定恢复**；但 284 帧的 `data/inputs/legacy_sources/` 只是其子集，
两者 header 集不一致，不能互相冒充。

**(C) 5925 / 5931 语料（历史 GICP 优势案例）**

`/home/ltc/flyagle/analyzer/lidar_ndt/audits/gicp_vgicp_benchmark/`（VERIFIED）

- `runs/` 775 MB 冻结运行；`FROZEN_RESULTS.csv`、`SEED_SENSITIVITY.csv`、`TRAJECTORY_RESULTS.csv`；
- `DATA_PROVENANCE.md`：Source 来自 `/home/ltc/flyagle/data/rosbag/gpu/gpu_0.mcap` 的
  `/sensor/lidar_fusion/points`；EKF seed 来自 `/media/ltc/T7/loop7_loc/loop7_loc_0.mcap`；
  参考为 `gnss_base.npy` / `raw_gps.npy` / `raw_attitude.npy`；
- `BAD_SEED_REPORT.md` 记录了 5925（small_gicp 恢复至 Y=533.175，CUDA-NDT 停在 525.909 错误解）
  与 5931（三方法全部失败）的**精确 Header** 和逐方法误差；
- `FINAL_COMPARISON.md`：3,546 帧 × 3 方法 × 3 重复 = 31,914 行，"逐帧Seed矩阵"已冻结。

这是唯一同时具备**独立 GNSS 工程参考 + 逐帧 Seed + 三后端结果**的语料，因此是
"5925 GICP 优势能否复现"的合法来源。**注意其时间基准与 loop44 完全不同**（Header 形如
`1791985909…`），不得与 loop44 拼接为一次连续运行。

### 5.3 Phase AW 的既有结论与配置身份（VERIFIED 于工作区 `ieskf-aw`）

`experiments/phase_aw_native_gicp_fusion/`：

- `build_gicp.sh`：`c++ -O3 -DNDEBUG -std=c++17 -I/usr/include/eigen3 -I$SMALL_GICP_INCLUDE gicp_service.cpp -ltbb`
- `gicp_service.cpp`：独立 JSONL stdin/stdout 服务，复用上游
  `PointCloud / KdTreeBuilderTBB / estimate_covariances_tbb / GICPFactor / ParallelReductionTBB /
  Registration + LM + rightSE3`；自定义 `RoiRejector`（`max_dist_sq = 16`，即 4 m 距离门限；
  另加 120 m 圆柱 + Z 区间拒绝）。
- `GICP_INTEGRATION.md` 明示参数：TBB arena 4、kNN 20、距离 4 m、max 50 iter、平移 1 mm、旋转 0.1°；
  无额外 Source/Target 降采样；Target 静态树+协方差只建一次；Source 每帧重建；
  **`inverseH` 不作为已标定 R**；GICP15D 基线使用固定工程 R（平移 σ=0.5 m、旋转 σ=1°）。
- `manifest.json`：`third_party_revision = fa0cfc98…`，`input_identity.target = 0895ad26…`（= 0105 Target），
  `gicp_lap frames = 1468 / 73.35 s / 1401 updates`，`gicp_multilap frames = 4134`。
- `FINAL_CONCLUSION.md`：CUDA-NDT 0.577 m / fixed-R NDT 0.612 m / GICP fixed-R 0.596 m；
  明确 **未** 验证方向性 R；明确 CPU GICP 在此工作负载 P95≈8.36 ms 但整环 P95≈26.97 ms，
  **严格 50 ms 未通过**。

### 5.4 KG-S3 发布成果（VERIFIED，远端 fetched）

`git show de651ab:experiments/phase_kg_s3/` 与
`/media/ltc/新加卷/autoeagle/phase_kg_s3/reports/`（16 个 md）。

- A 线：A2 = CUDA-NDT Score + 完整 15D MAP，保留条件 1.9762459 m 横向修正（old21，517.936350068 s），
  距保存的正确附近候选 0.004986 m；全程 75 ms 参考 XY RMSE 2.379/2.441 m vs A0 2.343/2.415 m（**无改善**）。
- B 线：`KINEMATIC_BK` 仅运动学隔离通过；相邻 XY 参考 RMSE 4.27782 → 0.15600 m，但纯预测链 155.55 m 漂移；
  当前局部 NDT 里程计 **NO-GO**，正式归档。
- C4_22：**认证于 bag 517.986342260 s / old profiling 396.999993344 s**，
  2499 点 Source、98213 点 Target、Seed、原 CUDA 错误候选约 4.506 m；
  但**无同帧成熟 15D prior/P**，故 A2 wrong-basin 结果 `NOT EXECUTED`。

→ 这条记录直接支撑本轮 G1 的定位：**G1 只做几何前端，不冒充 15D 闭环结论**。

---

## 6. 明确缺失与受影响阶段

| 缺失项 | 证据 | 受影响 | 处理 |
|---|---|---|---|
| 713.236 s 的完整真实输入前缀 | KG-S3 报告只做到 724.96 s 的"已核验前缀"，首个长回放被中断 | G1 的 713 案例 | **本轮不做 713**；在 `G1_VALIDATION_RESULTS.md` 标注 NOT COVERED，不用近似帧伪造 |
| 395.7–397.1 s 区间的**1013 图历史 Seed** | 历史 1013 运行的 Seed 未冻结为独立文件；只有 loop44 0105 运行的 `seed` | — | 用 0105 运行的 `seed` 同时喂两图（这正是任务书描述的"同 Source/Seed 改变地图"实验），并在报告中声明 seed 来自 0105 自身因果运行 |
| old21 (517.936 s) 帧的 Source | F_multilap 覆盖 273.6–480.2 s | G1 old21 案例 | **NOT COVERED**；只用 KG-S3 已发布的 1.976 m 条件结论作历史引用，不重跑 |
| Phase AW 的 1,468 帧 gicp_lap 原始 Source | `gicp_lap/` 只存 `*_poses.bin`（20 pose/块），Source 未随实验落盘 | G0 全圈复现 | 用 §5.2(B) 的 1,398 帧 legacy 语料做 G0 等价性校验（AW 也在同一语料上做过 `matcher_ab_fixed`） |
| `legacy_index.jsonl` 的绝对路径 | 指向已删除的 `lidar_ndt/audit/...` | 数据加载 | 通过 `analyzer/lidar_ndt/audits/native_covariance_20261024/.../captured_sources/` 重绑定，并逐帧校验 sha256 |
| loop44 独立 GNSS 真值 | bag 不含 GNSS 话题；AS 的 `evaluate_reference.py` 需要外部 `--gnss` CSV 与中心线 CSV | 绝对精度 | **不宣称绝对精度**；只用 (a) 相对同一 Seed 的 SE(3) 修正、(b) 与 CUDA-NDT 基线的姿态差、(c) 双图差分盆地行为 |
| `map_0105_voxel.pcd` 的 sha | 1.08 GB，本轮未重算 | — | 只用冻结 `map0105_target.bin`（已有 sha），不重新从 PCD 生成 0105 Target |

---

## 7. 复现入口（本轮新增）

- 研究 worktree：`/home/ltc/flyagle/ieskf-gicp15d`，分支 `dsh/gicp15d-pipeline`
- 报告目录：`experiments/phase_gicp15d_pipeline/`
- 大型输出：`/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/{artifacts,logs}`
- 数据根（只读）：`IESKF_DATA_ROOT=/home/ltc/flyagle/ieskf/data`

---

## 8. 审计后更正（逐字节复核结果）

本轮后续的独立逐字节复核修正/补充了 §4–§5 的若干条目，以下为准：

1. **0105 Target 的 PCD 来源**是 `map_0105_voxel_fix.pcd`（sha
   `8561cd2254ed35164134a3146e89dfcd403643193ff11c84db93fde2b77ad52a`，67,307,313 点），
   **不是** `map_0105_voxel.pcd`。生成脚本为
   `experiments/phase_ar_loop44_map0105/map_export.cpp`：`loadPCDFile` → PCL VoxelGrid
   leaf 0.8（XYZ，丢 intensity）→ 写 `<u64 count>` + 12 B/点。
   本次复核仍无法证明 `map_0105_voxel.pcd` 与 `_fix.pcd` 内容一致（未做全文件比对）。
2. **`<stamp>_poses.bin` 的格式**已确认：`float64`，`(N, 8)` 行 =
   `[dt_from_ref_s, qx, qy, qz, qw, tx, ty, tz]`，每帧一个文件（20–24 行，覆盖约 62.5 ms），
   **最后一行 `dt = 0`**，其平移与 `transactions.jsonl[i].seed` 的平移**逐位相同**
   （max abs diff = 0.0）。它不是 4×4 矩阵序列。
3. **参考轨迹的可评范围**：`phase_as_accuracy_quality/multilap/evaluation.npz` 的
   `valid` 只有 **1,484 / 4,134** 帧，覆盖 offset **278.124–362.423 s**。
   报告中引用的原始 `gnss_baselink_truth_delay75_full_validated.csv` 在本机**不存在**。
   因此本轮 `W_ONSET`（395.6–398.8 s）**没有任何真值**可用。
4. **5925 / 5931** 是 `analyzer/2026-09-30_.../analysis_full_lap/full_lap_frame_errors.csv`
   的 `frame_id`（stamp 1763205858.150005 / 1763205858.450005，offset1 438.286 / 438.586 s，
   `truth_valid = 1`），但其世界坐标是 `truth_x ≈ −296, truth_y ≈ 62`（`map_version 97`），
   与 0105/1013 的 seed 世界（`x ≈ 48, y ≈ 520`）**不是同一 map frame**，且未存储变换。
   → **不能用 loop44 语料复现**。
5. **713.236 s** 不在任何 ieskf 语料（最长 480.206 s）内；只在
   `auto-eagle-main/.../phase_f_full_loop44`（2612 帧，225.6–834.5 s），
   其 Source 为 `_source_y35.bin`（另一条预处理管线），Seed 为
   `seed_reconstructed_matrix`（重建）。→ 本轮 **NOT COVERED**。
6. `data/inputs/legacy_index.jsonl` 的 1,398 条路径**全部失效**（0/1,398 存在）；
   真实字节在 `analyzer/lidar_ndt/audits/native_covariance_20261024/phase_f1_f2_20261025/results/captured_sources/`
   （1,399 项含 `index.jsonl`，55 MB）。`data/inputs/legacy_sources/` 只是其中 284 帧的副本
   （offset 423.536–438.036 s）。
7. **`map_0105_voxel_fix.pcd` 之外的两个大 PCD 未做 sha 重算**：`map_0105_voxel.pcd`
   （1.08 GB）与 `merged.pcd`（3.5 GB）。本报告不对其内容作任何声称。
