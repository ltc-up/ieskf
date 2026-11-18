# FINAL_STATUS — Phase GICP15D 本轮

日期 2026-11-18。分支 `dsh/gicp15d-pipeline`（基于 `de651ab`，KG-S3 发布提交）。

## 一句话

G0 审计完成、G1 C++ 前端建成并在真实冻结点云上跑通、**与 Phase AW 基线数值等价**
（1,468 帧，平移差中位 6 × 10⁻⁶ m、p95 8.6 mm）；但**指定的 4.5 m 错误盆地未能在本批
冻结输入上复现**，因此 G1 判定 **PARTIAL**，G2/G3 **未实施**。

## 分项状态（严格区分）

| 陈述 | 状态 |
|---|---|
| 代码已实现 | **YES**：`src/gicp15d/`（6 个模块 + CLI + 测试），C++17，独立 CMake |
| 编译通过 | **YES**：g++ 13.3，`-Wall -Wextra` 无 error |
| CPU 合成测试通过 | **YES**：41/41（含合成刚性恢复到 0.0000 m / 0.00000 rad） |
| 真实冻结点云配准跑通 | **YES**：1,468 帧（G0 等价）+ 1,648 次（206 案例 × 8 preset） |
| 已有 GICP 基线复现 | **YES**：p50 6e-6 m，p95 8.6e-3 m，inlier 计数差中位 0 |
| 前端显示明确几何收益 | **NO / PARTIAL**：唯一正收益为 `SPECTRUM_FLOOR`（`d_ndt` p50 −21%），量级小；法向门限有害 |
| 指定 4.5 m 错误盆地复现 | **NO**：双图差 ≤0.76 m |
| 自身反馈连续定位 | **NOT IMPLEMENTED**（G2） |
| 真实时钟运行 | **NOT IMPLEMENTED** |
| 生产资格 | **NO**（且本轮未申请） |

## 本轮实际使用的源码与版本

| 组件 | 路径 | 版本 |
|---|---|---|
| 15D 研究仓库 | `/home/ltc/flyagle/ieskf` | 本地 HEAD `b143e88`（`research/phase-av-filter-improvement`），8 个 worktree；主工作区有用户未提交修改 `tests/test_surface_observation.py`（未触碰） |
| 研究基线 | `origin/codex/kg-s3-publication` | `de651abe789369c2f497324b700faa997be19bf8`（本轮 `git fetch` 获取） |
| 本轮 worktree | `/home/ltc/flyagle/ieskf-gicp15d` | 分支 `dsh/gicp15d-pipeline`，基于 `de651ab` |
| 生产仓库 | `/home/ltc/flyagle/auto-eagle` | 分支 `ndt_test`，HEAD `aa50e354`；**只读，零修改** |
| small_gicp | `auto-eagle/src/a2rl_3rdparty/small_gicp_vendor/upstream` | `fa0cfc98…`，MIT，未修改 |
| fast_gicp | `/home/ltc/flyagle/3rd_party/fast_gicp` | `0e7ec14…`（审计用，未链接） |
| FAR-LIO | `/home/ltc/flyagle/3rd_party/FAR-LIO` | `ef20510…`（审计用；`submodules/` 未初始化） |

## 真实数据

| 输入 | 身份 |
|---|---|
| Target 0105 | `ieskf/data/inputs/map_target.bin`，908,013 pts，sha `0895ad26…` |
| Target 1013 | `analyzer/…/ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin`，997,680 pts，sha `2f7c78d7…` |
| Source/Seed | `ieskf/data/phase_as_accuracy_quality/F_multilap/`（4,134 帧，offset 273.556–480.206 s），`transactions.jsonl.seed` |
| AW 对照 | `ieskf-aw/data/phase_aw_native_gicp_fusion/private/matcher_ab_fixed/matches.jsonl`（1,468 帧） |

## 未完成 / 无法证明（完整清单）

1. **395.7–397.1 s 的 4.5 m 盆地未复现**。Seed 未漂移、ROI 跟随 Seed、两图差异本身只有 0.48 m。
2. **5925/5931 未复现**：与 loop44 不属同一 map frame，无存储变换（`G1_FAILURE_ANALYSIS.md` F7）。
3. **713.236 s 不可达**：任何 ieskf 语料最长 480.206 s（F8）。
4. **old21（517.936 s）未覆盖**：超出语料范围；只引用 KG-S3 已发布的 1.953 m 条件结论。
5. **绝对精度无法评价**：参考只覆盖 1,484/4,134 帧（278.1–362.4 s），本轮窗口全在其外；
   原始 75 ms GNSS CSV 已不在磁盘。所有误差陈述都是代理。
6. **G0 与 AW 的 19 帧 >10 cm 偏差未归因**：没有做单线程全量复跑分离 OMP 归约顺序的影响。
7. **G2 / G3 未实施**：无一行 C++ 15D 代码被执行。
8. **严格 20 Hz / 50 ms 资格**：NOT_VALIDATED。`align_ms` p50 1.6–5.1 ms 只是配准本身；
   完整周期缺 Deskew、IMU 传播、15D 更新、ROS 调度。
9. **`robust_scale_adaptive` 未启用**；`SPECTRUM_FLOOR` 只测 `ρ=1e-3`；法向门限只测 60°。

## 下一步（有证据支撑的建议）

1. **先补 G1 的触发条件**：取得 1013 图下**真实漂移后的 Seed**，或 C4_22 的同帧成熟 15D prior/P。
   没有这个，任何"GICP 抵抗错误盆地"的声称都不可验证。
2. 只用 `SPECTRUM_FLOOR` 一个机制做**参数敏感性**（`ρ ∈ {1e-4, 1e-3, 1e-2}`）与跨窗口复验，
   在**未见窗口**上确认后再进 G2。
3. 5925/5931 走 `analyzer/lidar_ndt/audits/gicp_vgicp_benchmark/`（自带冻结 Source/Seed/Target
   与 GNSS 参考），而不是 loop44 语料。
4. 若要 G2：先做数值等价单测（Python 参考 vs C++），通过后才做连续回放。
