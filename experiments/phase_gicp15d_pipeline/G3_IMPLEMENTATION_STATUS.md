# G3_IMPLEMENTATION_STATUS

**状态：NOT IMPLEMENTED（本轮未实施）**

前置条件是 G2 已能稳定闭环。G2 未实施，因此 G3 不启动。本轮**没有**写任何健康性融合、
观测接纳、降权或信息映射代码。

## 本轮已具备的原料（G1 真实产出，可直接用于 G3）

`ResidualStatistics` / `InformationDiagnostics` 已在 1,648 次真实配准中逐帧落盘：

- 逐门限拒绝计数：`rejected_distance`、`rejected_region`、`rejected_reciprocal`
- 有效对应：`pairs`、`candidate_pairs`、`inlier_ratio`
- 残差：欧氏与 Mahalanobis 的 mean / median / p95 / max；IRLS 权重 mean / min
- 空间覆盖：12 扇区直方图、占用扇区数、主导扇区比例、归一化熵、距离中位/p95、近场比例
- 信息：6×6 H、`b`、cost、6 个特征值、条件数、最弱平移方向与比值、`rank_deficient`
- 多 Seed 探针：`--probe --probe-offsets 0,2,5,8,12`（本轮实现但**未在主实验启用**）

`information.covariance_status` 恒为 `"NOT_A_COVARIANCE"`——这是刻意的强制标签，
防止下游顺手把 H⁻¹ 当 R。

## G3 的设计约束（写给下一轮，避免越界）

1. **两种失效必须分开记录**：
   (a) 地图结构缺失 —— 低对应数、低扇区覆盖、弱/秩亏方向；
   (b) 错误盆地 —— 对应数高、残差低，但位置错。
   二者不能压成一个标量"健康分"。
2. **不默认 6 自由度同等可信**：用实测最弱方向（`weakest_translation_direction` /
   `weakest_translation_ratio`）做降权，且必须是显式可关闭配置。
3. **信息映射只允许可解释、保守、单调的形式**；禁止 `R = H⁻¹`。
4. **连续观测失效必须进显式状态**（COASTING / LOST）并记录进入与恢复事件，
   不允许静默维持高置信度。
5. **对照必须同输入、同 Seed、同 R 策略**，只切换健康逻辑。
6. 只有在 Pose 级融合被证明存在能力瓶颈后，才研究迭代 15D 几何融合；
   **不**重写 GTSAM 因子图或 FAST-LIO 架构。

## G3 为什么现在不做

即使 G1 的诊断字段已经够用，**没有 G2 就没有可被"降权/保持"影响的滤波器**。
在 G2 之前写 G3，只会产出一堆无法闭环验证的开关，属于"为了用先进算法而写算法"。
