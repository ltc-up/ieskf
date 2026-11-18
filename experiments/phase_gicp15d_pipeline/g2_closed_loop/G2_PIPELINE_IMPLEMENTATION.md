# G2_PIPELINE_IMPLEMENTATION

## 1. 真实调用链（已核实，非读 README）

```
run_loop.py (AW, 冻结)
  └─ exec run_closed_loop.py (AR, 字节未改)
       ├─ 载入 events(loop44.json) / bindings(bindingslap.json) / target(map_target.bin)
       ├─ PoseDelayReplay(x0,P0,held_imu,lever)       # 500 ms checkpoint + 重放
       ├─ 逐帧：
       │   ├─ advance(available)                      # 摄入 IMU/Wheel 到该帧可用时刻
       │   ├─ prior,Q,_ = checkpoint(ref)             # ref = native_reference_ns
       │   ├─ seed = pose(prior, lever)               # map<-base_link
       │   ├─ 写 {i:04d}_poses.bin（20 个 Deskew 快照）
       │   ├─ deskew_service (CUDA, 常驻) ──> <stamp>_A1.bin
       │   ├─ 请求 matcher: seed_common + roi + target
       │   │      [本轮由 gicp15d_g0_service 回答]
       │   ├─ advance(job_now + round(latency*1e6))   # <-- 依赖实测耗时
       │   ├─ event = {kind:'ndt', header_ns:ref, pose:T, R:fixed, correlation_group}
       │   └─ replay.ingest(event)                    # 回滚到 ref 重放
       └─ 输出 transactions.jsonl / states.jsonl / summary.json
```

EKF 数学仍在 Python（`src/geometry_ieskf`），**没有** C++ 实现。本轮没有把它搬到 C++，
因为任务要求的是"证明闭环成立"，而不是重写滤波器。

## 2. 新增 C++ 模块

| 文件 | 作用 |
|---|---|
| `src/gicp15d/src/g0_service_main.cpp` | 流式 JSONL 服务；`G0_AW_EQUIV` 预设；回答 `final/converged/candidate_valid/iterations/exit/pairs/candidate_pairs/euclid_median_m/maha_median/sectors_occupied/condition_number/seed_translation_m/align_wall_ms/...` |
| `GlobalMapManager::setRehashOnLookup(bool)` | 流式模式下跳过每请求 10.9 MB 目标 sha 重算（默认仍为 true，单测路径不变） |
| `GicpFrontend::setTargetRehashPerRequest(bool)` | 同上，前端层开关 |

CMake 新增目标 `gicp15d_g0_service`。单测 63 项保持全通过。

## 3. 请求/响应协议

请求（与归档 CUDA 服务同构）：`case_id, header, group, scene, source_path,
target_path, target_sha, roi[13], seed_common[4][4], q, max_iterations`。

`roi = [enabled, shape, cx, cy, cz, min_x, max_x, min_y, max_y, min_z, max_z, radius, generation]`，
G0 只用圆柱部分（`center=roi[2:5]`, `low=roi[9]`, `high=roi[10]`, `radius=roi[11]`），
与 Phase AW 的 `RoiRejector` 语义一致；`shape`/box 字段被忽略并在此声明。

响应必须含 `backend="small_gicp"`，否则驱动器抛 `MATCHER_BACKEND_IDENTITY_MISMATCH`。

## 4. G0 配置（冻结，未改）

small_gicp 点协方差 `(1e-3,1,1)`、kNN 20、距离门 4 m、LM 50 iter、1 mm / 0.1°、右扰动、
线程 4。**未**启用 FROBENIUS、鲁棒核、法向门、互惠检查、coarse-to-fine。

## 5. R 与门控（沿用 Phase AW，未改）

`pose_covariance` 被 `run_loop.py` 覆写为固定工程 R：平移 0.25 m²、旋转 `(π/180)² rad²`，
六自由度。观测以 `ExternalPose`（map 平移 + prior-anchored SO(3) Log）进入 15D。
**没有**用 GICP fitness 或 Hessian 构造 R。
