# FROZEN_PARAMETERS — 本轮全部冻结参数与身份

无私有路径、无机器身份、无标定数据。所有 sha 均为公开可复算的输入身份。

## Target 身份

| 标签 | sha256 | 点数 | 字节 |
|---|---|---:|---:|
| 0105 | `0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968` | 908,013 | 10,896,164 |
| 1013 | `2f7c78d7aa61d219033a3cb9bc1822204db450e9d3ce0405cd1bca9fdcd197ac` | 997,680 | 11,972,168 |

`source_sha256` 逐帧写入每条结果的 `source_sha256` 字段（由 C++ 重新计算）。

## 求解器（全部 preset 共用，除注明外）

| 参数 | 值 |
|---|---|
| 后端 | `small_gicp fa0cfc98` GICPFactor + LM + 右扰动 `T·exp(δ)` |
| 协方差 kNN | 20（含自身） |
| 最大对应距离 | 4.0 m（平方后 16.0 m²） |
| 迭代上限 | 50 |
| 平移收敛阈值 | 1.0e-3 m |
| 旋转收敛阈值 | 1.7453292519943296e-3 rad（0.1°） |
| 线程 | 8（报告数值）／1（确定性测试） |
| ROI | 圆柱 `r = 120 m`，`z ∈ [seed.z − 10, seed.z + 30]`，`center = seed.xyz` |

## 各 preset 的参数

| preset | 协方差 | ROI 裁树 | 法向门 | 互惠 | 鲁棒核 | coarse |
|---|---|---|---|---|---|---|
| `G0_AW_EQUIV` | UNIT | 否 | 关（180°） | 否 | 无 | 否 |
| `G0_SMALL_GICP_NATIVE` | UNIT | 否 | 关 | 否 | 无 | 否（20 iter） |
| `G1_SPECTRUM` | FLOOR ρ=1e-3 | 否 | 关 | 否 | 无 | 否 |
| `G1_ROI_CROP` | UNIT | **是** | 关 | 否 | 无 | 否 |
| `G1_NORMAL_GATE` | UNIT | 否 | **60°** | 否 | 无 | 否 |
| `G1_RECIPROCAL` | UNIT | 否 | 关 | **是** | 无 | 否 |
| `G1_ROBUST_GM` | UNIT | 否 | 关 | 否 | **Geman-McClure c=1 m** | 否 |
| `G1_ROBUST_HUBER` | UNIT | 否 | 关 | 否 | Huber c=1 m | 否 |
| `G1_COARSE` | UNIT | 否 | 关 | 否 | 无 | **1.5 m → fine，8 m / 30 iter** |
| `G1_FULL` | FLOOR ρ=1e-3 | 是 | 60° | 是 | GM c=1 m | 否 |

## 案例选择（不按结果筛选）

| 集合 | 规则 | 帧数 |
|---|---|---:|
| `W_ONSET_RECOVERY` | offset ∈ [395.606, 398.800] s | 63 |
| `CTRL_STRIDE100` | 全语料 `index % 100 == 0` | 40 |
| `phase_aw_matcher_ab_fixed` | AW 存储的 1,468 帧 | 1,468 |

## 明确未标定的量

- 观测协方差 R：G1 不产生 R。G2 设计中的固定值（0.25 m² / (π/180)² rad²）来自 Phase AW 的
  既有声明，**不是**本轮标定结果。
- GICP Hessian 逆：**不使用**，仅作 `NOT_A_COVARIANCE` 几何诊断。
- `robust_scale_adaptive`：字段存在，**未启用**。

## 复现环境

g++ 13.3.0、CMake 3.28.3、Eigen 3.4、OpenMP 4.5、PCL 1.14（仅审计用）、
`nlohmann/json`、32 线程 CPU、30 GiB RAM。CUDA 未参与本轮任何计算。
