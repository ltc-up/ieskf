# FAR_LIO_COVARIANCE_AUDIT

只读审计。FAR-LIO 版本 `ef20510316ad1f8dbf2e428e4a193f700493eccd`（本地树干净）。

## 1. 源码位置

| 路径 | 作用 |
|---|---|
| `lib_cpp/odometry_cpp/modules/map_handler/include/map_handler/map_handler_base.hpp:228-236` | 解析 `map.cov_regularization` 字符串 → 枚举 |
| 同文件 `:345` | 调用 `set_normal_covariance(..., num_neighbors, cov_regularization_type_)` |
| 同文件 `:382-421` | **CPU** 实现：SVD / FROBENIUS / MIN_EIGENVALUE 三分支 |
| `.../map_handler/map_utils.cuh:87-95` | **GPU** 实现，FROBENIUS 与 CPU 逐行相同 |
| `config/far-lio.yml:160` | `cov_regularization: "FROBENIUS"` |
| `config/far-lio.yml:158` | `voxel_size: 4.0` |
| `odometry_config.hpp:186-192` | `NUM_NEIGHBORS = 10`（GICP_EXT / CUDA_GICP_EXT） |

## 2. 实际公式（逐字）

```cpp
case types::CovRegularizationType::FROBENIUS: {
  float lambda = 1e-3;                                   // float, 不是 1e-3f
  Eigen::Matrix3f C = cov + lambda * Eigen::Matrix3f::Identity();
  Eigen::Matrix3f C_inv = C.inverse();
  Eigen::Matrix3f C_norm = (C_inv / C_inv.norm()).inverse();  // norm() = Frobenius
  neighbor.point->cov = C_norm;
  break;
}
```

要点：

1. `cov` 是 kNN 邻域的**有偏**样本协方差（除以 n，无 Bessel）。
2. `Eigen::Matrix3f::norm()` 在矩阵上是 **Frobenius 范数**，不是谱范数。
3. 因为 `C0 = cov + λI` 正定，`C_far = (C0^-1/||C0^-1||_F)^-1 = ||C0^-1||_F · C0`。
   即 FROBENIUS **等价于把 C0 乘上一个标量**，方向不变、幅值被归一化。
4. CPU 与 GPU 完全一致（逐字相同代码）。
5. 与 small_gicp 的 `CovarianceSetter`（特征值替换为 `(1e-3, 1, 1)`）**本质不同**：
   SVD 法是丢弃实测谱并按法向/切向重建；FROBENIUS 法保留实测谱的相对形状，
   只做正则化 + 尺度归一化。
6. 与 `SPECTRUM_FLOOR`（`λ_i ← max(λ_i, ρλ_max)`）也不同：后者保留绝对尺度、只压低下界；
   FROBENIUS 保留形状、**重标定**尺度。

## 3. 三个必须区分的东西

| 概念 | 所在层 | 本轮的用法 |
|---|---|---|
| GICP 内部点协方差（3×3） | 配准目标函数 | 本轮新增 `FAR_FROBENIUS` 模式 |
| GICP 位姿 Hessian（6×6） | 优化器输出 | 仅诊断，`NOT_A_COVARIANCE` |
| 15D 观测协方差 R（6×6） | ESKF | **未使用**，也未改动 |

## 4. 本机实现的差异（已声明）

| 项 | FAR-LIO | 本轮 `G1_FAR_FROBENIUS` |
|---|---|---|
| 精度 | `float` (Matrix3f) | `double` (Matrix3d) |
| λ | 字面 `1e-3` | 常量 `1.0e-3` |
| kNN | 10（体素内邻域，体素 4.0 m） | 20（= G0，为保持单变量对照） |
| 邻域不足 | `cov = 0`（零矩阵） | 与上游一致：各向同性 Identity，并计数 |
| 法向 | 同一次 PCA 一并输出 | 同 |
| 退化保护 | 无额外检查 | 结果非有限或非正定 → 回退 Identity 并计为 `insufficient` |

**因此本轮不是"FAR-LIO 复刻"，而是"FAR-LIO 的 FROBENIUS 公式在本前端参数下的一次受控移植"。**

## 5. 数值一致性验证（单测，已通过）

`tests/run_tests.cpp::testFarFrobeniusRegularization`：

1. 对角协方差闭式对照：`C_far == ||(C+λI)^-1||_F · (C+λI)`，误差 `<1e-12`；
2. 与"字面 inverse/normalise/inverse 表达式"独立求值对照，误差 `<1e-12`；
3. 旋转等变性 `C_far(RCRᵀ) == R C_far(C) Rᵀ`（相对 `1e-13`）；
4. 退化平面（秩 2）：有限、对称、正定，且**法向仍是最小轴**（`|eigvec·n| > 0.999`）；
5. 近零协方差与秩 1 协方差：有限且正定；
6. 非有限输入 → 返回 Identity（拒绝而非传播）；
7. **G0 不受影响**：同一云上 `SMALL_GICP_UNIT` 的谱仍精确为 `(1e-3,1,1)`。

单测总数 48 → **63**，全部通过。
