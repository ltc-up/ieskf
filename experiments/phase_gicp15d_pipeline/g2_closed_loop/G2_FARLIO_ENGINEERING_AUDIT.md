# G2_FARLIO_ENGINEERING_AUDIT

只读审计。FAR-LIO `ef20510316ad1f8dbf2e428e4a193f700493eccd`，工作树干净，
`submodules/TAM__common` **未初始化**（空目录）⇒ 所有 `tum_helpers_cpp` /
`ros2_watchdog_cpp` 符号标 NOT FOUND。未构建、未修改。

目标系统是 CPU-only 的 15D ESKF + small_gicp + 固定 PCD 地图。FAR-LIO 的 9 态 EKF
**不是**替换候选，只借工程机制。

## 1. Seed 与时间对齐

**Seed = EKF 在 LiDAR `header.stamp` 处的位姿，经 TF 取得。**
`tam_odometry.hpp:93-132` `cloud_callback`：`lookupTransform(odom, child, msg->header.stamp)`；
失败回退 `tf2::TimePointZero`（最新值）；再失败用上一帧 odometry 并置 `valid_guess=false`
→ `ExternalGuess::set_pose` 做 `T·T_prev⁻¹·T` 的恒速外推（`model_utils.hpp:28-31`）。
发布侧 `state_estimation_node/node.hpp:395-396,481-501` 以 `now − average_delay_ms (10 ms)`
的 100 Hz TF 输出。**没有任何检查确认 EKF 状态比帧更新**。

前端**不做**延迟补偿，输出戳就是 `header.stamp`（`odometry_pipeline_impl.hpp:233`）。
补偿在融合侧（`node.hpp:612-627`）：`stamp += average_input_delay_ms` 当
`stamp − additional + average ≤ now`，再 `DelayCompensation::compensate(input, now − stamp)`
并按 `R·(1 + virtual_scale·Δt)`，`virtual_scale = 10.0`（`far-lio.yml:80`）。
`DelayCompensation` 实现在未初始化的 submodule 里 ⇒ **NOT FOUND**。

**判定**：PORT（S）。风险：TF 采样不足时会静默退化到"最新值"，把一整帧延迟当作位姿误差注入且不上报。

## 2. 更新流程与线程

`cloud_callback` 同步调用 `pipeline_->register_frame` 并在回调内发布
（`tam_odometry.hpp:145-149`）——无工作线程、无输出双缓冲。
EKF 侧 `measurement_callback` 只存测量，真正融合在 100 Hz 定时器
`model_update_callback → state_estimation_->step()`（`node.hpp:303-314,367-378`）。
只有 `MapHandler::mutex_` 与 `std::atomic<bool>`；`state_estimation` 内部无锁，
依赖单线程执行器。

**判定**：未移植。风险：单执行器下 GICP（最长 200 ms）会阻塞看门狗与地图回调。

## 3. 延迟与超时

三层：点云看门狗 500 ms（`node_base.hpp:383,390`，每 100 ms 检查）；配准
`frame_outdated = 100 ms` 只发 WARN（`far-lio.yml:141`）；测量超时 500 ms →
`state_machine_->set_position_invalid()`。另有 `max_time_diff = 50 ms` 位姿-初值时差检查、
Deskew `30 ms` / `vel ≥ 3.0 m/s` 门槛、求解器 `max_time = 200 ms`。

**判定**：ADAPT（M）。注意 `max_time 200 ms > frame_outdated 100 ms`，
即 100–200 ms 的配准按设计就是"WARN 但照用"。

## 4. 失败匹配的处理

`DiagnosticHandler::check_registration`（`diagnostic_handler_base.hpp:149-167`）：
`!converged → ERROR`；`converged && duration > 100 ms → WARN`。**失败位姿仍然**：
发布（`pipeline:234`）、写入 `odometry_`（`:256`）、喂给 `AdaptiveThreshold`
（`:205`，会污染 `model_sse_` 且无重置）、并且——尽管注释说"仅成功时"——
**仍然插入地图**（`:211-229`，无状态判断）。

EKF 侧：WARN **被视为有效**并参与融合（`state_machine.hpp:197-209` 只把 ERROR/STALE 置无效）；
残差是**截断**而非拒绝（`outlier_rejection.hpp:71-76`）。

**判定**：PORT 其契约（状态 + valid bit），但必须改成 **WARN → 降权、ERROR → 跳过**；
**SKIP** 残差截断（会保留错误测量的偏置切片）与"锁存上一帧测量每 10 ms 重复融合"（重复计信息）。

## 5. 固定地图管线与地图缓存

`far-lio.yml:113` `update_map: true`、`input_map: ""` ⇒ **发布配置根本不用固定地图**，
地图从帧增量构建。固定地图分支存在：`input_map` 以 `.pcd` 结尾 → PCL 载入
（`utils.hpp:254-280`）；否则走 `tum_map_msgs` 服务，5 s 重试（`node_base.hpp:363-365`）。
双缓冲 `map_a_/map_b_` + `request_map_switch()/consume_map_switch()`，
在 `register_frame` 开头消费（`:84`）——新地图下一帧生效，绝不中途换。

已知缺陷：`update_in_progress_` 在 `service_is_ready()` 检查**之前** CAS，
且该失败路径不清零 ⇒ 一次早触发后地图更新永久卡死（`node_base.hpp:175-195`）；
`debug_.map_update` 由 detached 线程写、执行器读。

**判定**：PORT 模式（M）；**SKIP** 服务协议与 detached 线程（固定地图无重载需求）。

## 6. 预处理

地图：`tsl::robin_map`，`voxel_size 4.0 m`，每体素上限 40，最小间距
`get_resolution() = sqrt(4²/40) = 0.632 m`；法向/协方差在插入时算一次，
`NUM_NEIGHBORS = 10`；`max_distance 1000 m` 半径淘汰；静态地图不淘汰。
`cov_regularization: FROBENIUS`（`lambda 1e-3`）——**本项目已用真实数据证伪**（G1-C）。
帧：`crop_range/crop_footprint` 因配置是单元素列表而**全部关闭**；
`downsample: false` ⇒ 单次 `voxel_downsample` 每体素留一个任意点。

**判定**：**SKIP** 体素地图容器（用 small_gicp）；PORT 裁剪（S）。

## 7. 鲁棒核、门限与求解器

Cauchy `w = 1/(1+‖r/s‖²)`，`s = σ/3`；`σ` 来自 `AdaptiveThreshold`
（`model_sse` 初值 1.0，`σ = sqrt(sse/n)`，**硬上限 10 m**，采样门限 `3·0.25 = 0.75 m`），
关联门 `3σ ∈ [0,30] m`。求解器 GN，`max_iter 500`、`max_time 200 ms`、
收敛 `5e-3`、`INVALID` 于 `‖dx‖ ≤ 1e-12`。
Huber 分支比较 `|residual.squaredNorm()|` 与未平方的尺度 —— **单位不一致，移植需修**。

**判定**：PORT 自适应门限（S，但 10 m 上限与 0.75 m 采样门是为 4 m 体素调的，
细地图需重调，且一次发散会永久污染 `model_sse_`）；Cauchy 需自加（small_gicp 只有
Huber/Geman-McClure），`s = σ/3 ≈ 0.2 m` 在稀疏固定地图上会过度降权。

## 8. CPU vs GPU

两者都实现，`BUILD_CUDA` 编译期选择。GPU-only：`cuco` 静态地图（`2^23`，负载因子 0.3）、
设备端对应搜索、`swap_map` kernel、设备端 `build_linear_system`。
**算法分歧**：CPU 传 `squaredNorm` 并用 `threshold²` 过滤，GPU 存 `sqrt` 并用
`threshold` 过滤；GPU 精度项写 `map.cov + frame.cov.transpose()`（CPU 为 `+ frame.cov`）；
CPU 在 `correspondences.empty()` 时 `break` 的频率远高于 GPU ⇒ **`converged` 跨后端不可比**。

**判定**：SKIP。**不要用 FAR-LIO 的 GPU 数值校验 CPU 移植。**

## 9. 线程调度与输出时序

单执行器；`tbb::global_control(max_allowed_parallelism = num_threads=1)`；
无丢帧策略；`buffer_z` 可在 `far-lio.yml` 开启（用上一好位姿的 z 覆盖 seed 的 z）。

**判定**：ADAPT（S）：2 线程互斥执行器（点云一个、看门狗一个）。

## 10. 移植判定表

| # | 机制 | 判定 | 成本 | 主要风险 |
|---|---|---|---|---|
| 1 | Seed 取帧戳 TF + 显式回退链 | PORT | S | TF 不足时静默用最新值，延迟变成位姿误差 |
| 2 | 延迟补偿记账 | ADAPT | M | 实现不在树内；`R·(1+10Δt)` 无界增长 |
| 3 | 状态/valid-bit 通路，WARN≠OK | PORT | S | 必须比 FAR-LIO 更严 |
| 4 | 残差截断 | SKIP | S | 保留错误测量的偏置 |
| 5 | 锁存测量 10 ms 重复融合 | SKIP | S | 重复计信息 |
| 6 | 自适应关联门限 `3σ` | PORT | S | 上限/采样门需重调；污染不可恢复 |
| 7 | 椭球/时差门控 | ADAPT | S | 4/5 项在发布配置里是关闭的 |
| 8 | Cauchy 核 `σ/3` | ADAPT | S | 需自加；`s≈0.2 m` 过度降权 |
| 9 | 求解器上限（含 `max_time`） | PORT | S | 无 |
| 10 | 双缓冲地图 + 帧首切换 | PORT | M | 别复制 `update_in_progress_` 卡死 |
| 11 | PCD/服务地图载入 | SKIP | S | 服务协议与"删掉给的文件"契约 |
| 12 | 体素哈希地图 | SKIP | L | 0.63 m 分辨率会主导误差 |
| 13 | 帧体素降采样 | ADAPT | S | first-wins 任意 |
| 14 | 距离/足迹裁剪 | PORT | S | 单元素列表会静默关闭 |
| 15 | TF 位姿历史 Deskew | ADAPT | M | 依赖 100 Hz TF；否则静默 no-op |
| 16 | detached 异步地图重载 | SKIP | S | 卡死 + 数据竞争 |
| 17 | 单执行器 + 100 ms 看门狗 | ADAPT | S | 200 ms 配准阻塞一切 |
| 18 | `buffer_z` | PORT | S | 会掩盖垂向失败 |
| 19 | GPU 路径 | SKIP | — | 与 CPU 数值不可互换 |
| 20 | 9 态 EKF / StateMachine | SKIP | — | 明确超范围 |

## 11. 本轮实际移植了什么

**没有。** 只做判定。原因见 `G2_FAILURE_ANALYSIS.md` F6：在回放器不确定性修复前，
任何移植都会与噪声混在一起而无法归因。
