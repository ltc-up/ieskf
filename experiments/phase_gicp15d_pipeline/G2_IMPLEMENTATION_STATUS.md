# G2_IMPLEMENTATION_STATUS

**状态：NOT IMPLEMENTED（本轮未实施 C++ 15D）**

按任务书 §六：G2 只能在 G1 已显示明确几何收益后推进。本轮 G1 判定为 **PARTIAL**
（`G1_VALIDATION_RESULTS.md` §6），因此**没有**启动 G2 大规模实现或闭环实验。

## 本轮已完成的部分（接口与合同，非实现）

1. 15D 语义审计：状态布局 `[p(3), v(3), R, bg(3), ba(3)]`、右扰动、`inject` / `chart_map`、
   `F/G/Q` 的实际公式与数值（`gyro_nd 0.005`、`accel_nd 0.1`、`gyro_bw 1e-4`、`accel_bw 1e-3`、
   `gravity [0,0,-9.80665]`、`max_imu_interval 0.025`）、
   Joseph + 单次 `B·P·Bᵀ`、`valid_prior` 的对称性/Cholesky 检查、500 ms 延迟重放的拒绝码。
2. 观测接口合同：`ExternalPose` 的 6 行残差（base 平移 + `Log(R_priorᵀ R_meas)`）、
   `H[:3,:3]=I`、`H[:3,6:9]=R·[lever]×`、`H[3:,6:9]=I@anchor`。
3. R 合同：初始只用**已声明**的固定工程值（平移 0.25 m²、旋转 `(π/180)² rad²`），
   来自 Phase AW 的既有声明；**不**使用 GICP Hessian 逆，**不**使用 Native 协方差。
4. 数值等价清单（逐条，已写入 `ARCHITECTURE_DECISION.md` §Phase G2 与 15D 审计）：
   `jr` 的小角分支阈值 `a<1e-5` 与 Taylor 系数；`jr(ωdt/2)`（用于 `Abg`）与
   `jr(ωdt)`（用于 `Fθb`/`Gθg`）的**两个不同参数**；每子步 matrix→quat→matrix 重正交；
   事件驱动子步长（不是固定 100 Hz）；int64 ns 与 `round(dt*1e9)`；
   `cho_solve` vs `P Hᵀ/S`；`var = variance + 0.5²` 在 `H P Hᵀ` **之后**加入 `S` 并以
   `var·K Kᵀ` 进入 Joseph。
5. 阶段门：`(a)` 数值等价单测 → `(b)` 短时事件合同 → `(c)` 冻结输入连续回放 → `(d)` 真实时钟。
   四者不得互相冒充。

## 未实施的具体清单

- `gicp15d_eskf` 目标与 15×15 传播/更新代码：**未写**。
- 与 Python 参考的逐帧数值对齐：**未做**。
- 冻结输入连续回放：**未做**。
- 真实时钟：**未做**。
- 自身反馈闭环（状态→Deskew/Seed→GICP→15D）：**未做**。

## 明确不能声称

- 不能声称"GICP 已接入 15D"——那是 Phase AW 的既有成果，不是本轮新增。
- 不能声称"G2 部分通过"——没有任何一行 C++ 15D 代码被执行。
- 不能声称"CPU 单测通过"——G2 连单测都没有。
