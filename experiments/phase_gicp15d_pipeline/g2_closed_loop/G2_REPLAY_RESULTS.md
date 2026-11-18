# G2_REPLAY_RESULTS

## 1. 运行身份

| 项 | 值 |
|---|---|
| 驱动 | `ieskf-aw/experiments/phase_aw_native_gicp_fusion/run_loop.py`（冻结，未改） |
| 被 exec | `experiments/phase_ar_loop44_map0105/run_closed_loop.py`（字节未改） |
| 匹配器 | 本轮 `gicp15d_g0_service`（G0） |
| Deskew | `data/phase_ar_loop44_map0105/runtime/deskew_service`（CUDA，常驻） |
| 输入事件 | `data/phase_ao_ap_validation/loop44.json`（392,409 事件） |
| 绑定 | `data/phase_ar_loop44_map0105/bindingslap.json`（1468 帧） |
| Target | `data/inputs/map_target.bin`，sha `0895ad26…`，908,013 点 |
| 运行目录 | `/media/ltc/新加卷/autoeagle/phase_gicp15d_pipeline/g2_closed_loop/lap_g0` |

## 2. 闭环有效性（Q1 的证据）

| 指标 | 值 |
|---|---|
| 处理帧数 | **1468 / 1468** |
| 时长 | 73.350073732 s |
| `own_causal_seed` | **true** |
| 观测接纳 | **1401**（`EXTERNAL_POSE_UPDATE`） |
| 丢弃 | 67（`NDT_NOT_CONVERGED_OR_INVALID`） |
| 最大接纳间隔 | 0.250030859 s |
| 最大无观测时长 | 0.250030859 s |

`own_causal_seed=true` 表示每帧 Seed 来自本运行自己的 15D 预测（`checkpoint(ref)`），
不是历史 NDT 轨迹。结合 §3 的逐帧一致性，可确认这是**真实自反馈**：
第 k 帧的更新进入了第 k+1 帧的先验。

## 3. 对 Phase AW 的复现（Q2）

同 1468 帧、同输入、同驱动，比较本轮 G0 与归档 `gicp_lap`：

| 指标 | 值 |
|---|---|
| matcher `final` 平移差 均值/中位/p95/max | **0 / 0 / 0 / 0 m**（逐位相同） |
| 旋转差 p95 / max | 2e-6° / 6e-6° |
| 收敛标志一致 | **1468 / 1468** |
| 接纳决策分歧 | **0** |
| 后验 `p` 差 p50 / p95 / max | **0.153 / 0.497 / 5.847 m** |

**匹配器完全复现；后验不复现。** 这一组合正是 §5 缺陷的指纹：
观测逐位相同，而 EKF 轨迹因回放时钟推进不同而分叉。

## 4. 精度（相对、不可引用绝对值）

用归档 `gicp_lap/evaluation/evaluation.npz` 的 `G_0.075` 与 `historical_valid_0.075`
（1197 帧）对同一批帧打分，杆臂取 `base_link→imu_bosch` 合成 `(1.465, 0.172, 0.0403)`：

| 运行 | RMSE | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| 本轮 G0 闭环 | 3.369 | 3.183 | 5.078 | 6.377 | 10.027 |
| AW GICP 闭环 | 3.157 | 3.015 | 4.839 | 5.483 | 6.839 |
| NDT+EKF 闭环 | 3.317 | 3.110 | 4.835 | 6.137 | 9.113 |
| AW「NDT+同 R」控制 | 3.042 | 2.918 | 4.506 | 5.061 | 5.831 |

**绝对尺度不可引用**：第 3 位数字（3 m 级）说明我的对齐实现与签核过的
`evaluate_reference.py` 全管线（AW 报告 0.596 m）不一致，差一个近似常数项。
因此本表**只能用于排序**，不能用于精度结论。
而排序本身也落在噪声内：四者极差 0.33 m，回放器抖动 0.15 m 级。

## 5. 匹配器耗时（1401 帧，4 线程）

| 指标 | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| `align` (ms) | 5.72 | 12.53 | 20.32 | 32.78 |
| 单帧 `available→complete` (ms) | 19.89 | 31.24 | — | 206.72 |
| 整帧 `wall_ms`（summary） | 20.12 | 30.36 | 47.05 | 117.29 |

对应点中位 3,514。20 Hz 周期 50 ms：**p95 30.4 ms 有余量，p99 47.1 ms 贴近上限，
max 117 ms 会丢 1–2 帧**。未达"严格每帧 50 ms"。
