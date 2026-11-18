# THREE_STAGE_IMPLEMENTATION_PLAN

## Phase G1 — 可靠 GICP 几何前端（本轮完成，PARTIAL）

**目标问题**：地图局部缺失或 Seed 偏离时，更完整的 GICP 几何模型能否比 CUDA-NDT 提供更稳健的
"正确附近"匹配？

**已实现**（`src/gicp15d/`，C++17，独立 CMake）：

| 模块 | 文件 | 语义 |
|---|---|---|
| `GlobalMapManager` | `global_map_manager.{h,cpp}` | 一次性加载 + sha 校验 + 全图 kd-tree + 全图协方差；ROI 裁出**独立子点云与独立树**，按 `(sha, generation)` 缓存 |
| `GicpFrontend` | `gicp_frontend.{h,cpp}` | 8 个命名 preset；可选 coarse-to-fine；可选多 Seed 探针；输出完整诊断 |
| `RegistrationDiagnostics` | `registration_diagnostics.{h,cpp}` | 最终位姿**重新线性化**后的残差分布、12 扇区覆盖、6×6 H 谱/条件数/最弱平移方向 |
| `RobustGICPFactor` | `robust_gicp_factor.h` | 与上游同残余/同精度；额外 IRLS 权重（米制尺度）与法向一致性门 |
| `GicpRejector` | `correspondence_rejector.h` | 距离门 + ROI 门 + 互惠检查，分别计数 |
| `local_covariance` | `local_covariance.{h,cpp}` | `SMALL_GICP_UNIT`（= 上游精确复刻）与 `SPECTRUM_FLOOR` |
| CLI | `frontend_main.cpp` | JSONL 进 / JSONL 出，**逐帧求解完全在 C++ 内**，无 Python |

**preset**：`G0_AW_EQUIV`、`G0_SMALL_GICP_NATIVE`、`G1_SPECTRUM`、`G1_ROI_CROP`、`G1_NORMAL_GATE`、
`G1_RECIPROCAL`、`G1_ROBUST_GM`、`G1_ROBUST_HUBER`、`G1_COARSE`、`G1_FULL`。
每个 preset 都是"G0 设置 + 单一机制"，因此单机制消融是配置级开关而不是另一份代码。

**验收口径**：见 `G1_VALIDATION_RESULTS.md`。核心是双图差分（同 Source / 同 Seed / 换 Target），
而不是绝对精度。

**本阶段遗留**：`SPECTRUM_FLOOR` 只测了 `ρ = 1e-3`；法向门限只测了 60°；互惠检查只测了全局开启；
没有做参数扫描（任务书禁止在看到参考误差后调参）。

---

## Phase G2 — 完整 C++ 15D ESKF 融合（本轮未实施）

**前置条件**（本轮已产出，未执行）：G1 必须先在真实数据上显示可归因的几何收益。
本轮 G1 结论为 PARTIAL（§`G1_VALIDATION_RESULTS.md`），**未达到进入 G2 大规模闭环的门槛**，
因此只完成接口与等价性合同，不写连续回放。

**设计（已定，不实施）**

1. 独立目标 `gicp15d_eskf`，与 `gicp15d_core` 同仓，不链接 ROS。
2. 15 维状态 `[p(3), v(3), R, bg(3), ba(3)]`，**右扰动**，`inject: R←R·Exp(δθ)`，
   reset `B = diag(I, I, Jr(δθ), I, I)`。
3. 传播必须复刻 `inertial.py:model`：ZOH 偏置、`Eh=Exp(ω dt/2)` 求加速度、`E=Exp(ω dt)` 求姿态、
   `p += v dt + ½ a dt²`、每子步做一次 matrix→quat→matrix 重正交、`y.t = x.t + round(dt·1e9)`。
4. `Q = diag(repeat([σg²/dt, σa²/dt, σbg²·dt, σba²·dt], 3))`，`P = F P Fᵀ + G Q Gᵀ` 后对称化。
5. 观测用 `ExternalPose` 语义：6 行 `[map translation of base, Log(R_priorᵀ R_meas)]`，
   `H = [[I, R·[lever]×], [0, I]]`；初始 R 用**已声明的固定工程值**（0.25 m², (1°)²），
   **不**用 GICP Hessian。
6. Joseph + 单次 reset，保留完整 15×15 与交叉项，禁止 gain 裁剪 / P 膨胀 / 协方差清零。
7. 延迟：500 ms checkpoint + 按 `(header_ns, kind_rank, message_id)` 重放，拒绝 `LATE`/`IMU_GAP`/
   `DUPLICATE`/`OUTSIDE_CHECKPOINT_HISTORY`，语义与 `ndt_global/delayed_local.py` 一致。
8. 验收顺序：**(a)** 数值等价单测（与 Python 逐位/1e-12 级对比）→ **(b)** 短时事件合同测试
   → **(c)** 冻结输入连续回放 → **(d)** 真实时钟。四者不得互相冒充。
9. 数值等价风险清单已列于 `AUDIT_ARCHITECTURE_COMPARISON.md` 与 15D 审计（`Jr` 小角分支 `a<1e-5`、
   `jr(ωdt/2)` 与 `jr(ωdt)`, 事件驱动子步长、int64 ns、`cho_solve` vs 除法等）。

**G2 为什么本轮不做**：G1 未显示"正确附近匹配"的可归因增益，先叠加 G2 会掩盖前端的真实贡献，
违反任务书"避免在尚未有效的前端上继续叠加复杂后端"。

---

## Phase G3 — 几何健康性与可靠观测融合（本轮未实施）

**前置条件**：G2 已能稳定闭环。本轮 G2 未实施，故 G3 不启动。

**已具备的原料**（G1 实际产出，可直接用）：

- 逐门限拒绝计数（距离/ROI/互惠/法向）
- 有效对应数、inlier 比例、欧氏与 Mahalanobis 残差 mean/median/p95/max
- 12 扇区直方图、占用扇区数、主导扇区比例、熵、近场比例
- 6×6 H 谱、条件数、最弱平移方向与比值、`rank_deficient`
- 多 Seed 探针的收敛区域（`probe_offsets_m`）

**设计约束（写在前面，防止越界）**

1. 健康状态必须**分别**记录"地图结构缺失"（低对应、低覆盖、弱方向）与"错误盆地"
   （高对应、低残差但位置错）——两者不能共用一个标量。
2. 不默认 6 个自由度同等可信：先用**实测**最弱方向做降权，且降权必须是**可关闭**的显式配置。
3. 信息映射只允许"可解释、保守、单调"的形式；禁止把 Hessian 逆当已标定 R。
4. 连续观测失效必须进入显式状态（COASTING/LOST），并记录恢复事件，不能静默维持高置信度。
5. 与 G2 的对照必须同输入、同 Seed、同 R 策略，只切换健康逻辑。
6. 只有在"Pose 级融合被证明存在能力瓶颈"后才研究迭代 15D 几何融合；
   **不**重写 GTSAM 因子图或 FAST-LIO 架构。

---

## 阶段门（本轮实际判定）

| 门 | 判据 | 结果 |
|---|---|---|
| G1 代码可编译 | 独立 CMake，无新增依赖 | PASS |
| G1 CPU 合成测试 | 刚性恢复 + 各机制单测 | PASS（41/41） |
| G1 G0 复现 | 与 Phase AW 1,468 帧同输入对比 | PASS（p50 6e-6 m，p95 8.6e-3 m） |
| G1 真实冻结点云配准 | 206 案例 × 8 preset 全部运行 | PASS（无失败帧被丢弃） |
| G1 明确几何收益 | 双图差分出现可归因的盆地改善 | **FAIL / PARTIAL**：395.7–398.8 窗口未复现 4.5 m 盆地分裂 |
| G2 C++ 15D | 数值等价 + 连续闭环 | NOT IMPLEMENTED |
| G3 健康融合 | 在 G2 上的增量 | NOT IMPLEMENTED |
