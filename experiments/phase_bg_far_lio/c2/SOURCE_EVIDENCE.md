# 源码证据索引

CONFIRMED：研究HEAD62eb947、FAR ef205103（缓存clean）、TAM common 4b19b608、生产源码aa50e354。准确SHA见manifest；生产既有untracked目录未触碰。无运行中部署版本等价性推断。

| 结论 | 实际入口（相对各仓库） |
|---|---|
| FAR九状态名义方程与A/B/Q | FAR `lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf_3d.hpp`，名义方程约98行、A组装204行、Q_lin264行；vy相关导数156行 |
| FAR更新/残差裁剪/P | 同目录 `ekf.hpp`，K约76行、裁剪82行、P101行；对应helper/outlier_rejection.hpp |
| FAR R floor/limit/decay | 同目录 `base.hpp::update_measurement_covariance_matrix`74行、set_covariance_matricies142行 |
| FAR低速Bias学习、倾角 | `state_estimation.hpp::step`、`update_imu_offsets`1302行；`submodules/imu_handler.hpp::update_input_vector`；`ref_orientation_handler.hpp::update` |
| wheel激活条件 | `state_estimation.hpp::set_input_wheelspeeds`；`submodules/vehicle_model_handler.hpp::get_status`及默认STALE |
| persistent received缓存 | FAR `ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/node.hpp::model_update_callback`320行；set true627行；struct在types.hpp96行。全文未发现消费后设false |
| 时间戳、补偿与debug | node.hpp395行time_pub，613行measurement callback，552行两类compensated debug |
| 自身历史位移补偿 | TAM common `tum_helpers_cpp/src/delay_compensation.cpp::compensate`，固定TS索引、FIFO填充与超界 |
| 15D预测/误差图 | 研究 `src/geometry_ieskf/inertial.py::model/predict`；`state.py::inject/chart_map` |
| 15D Wheel与外部pose | `wheel.py::model/correct`；`external_pose.py::residual/correct` |
| Legacy及Native | `ndt_pose_observation.py::engineering_R`；`pose_uncertainty.py::native_pose_R`（本轮未启用Native更新） |
| C1真正运行路径 | `experiments/phase_bg_far_lio/c1/run_continuous_15d.py`30行ExternalPose、54行Legacy R、57行六维扩展、68行初始化 |
| 15D延迟及生命周期差别 | `src/ndt_global/delayed_local.py::ingest`；`pose_delay.py::step`；`managed_replay.py::LegacyComparisonReplay`120行，retract明确禁用 |
| 生产队列/历史/输出 | 生产 `ekf_localizer/include/ekf_localizer/aged_object_queue.hpp::pop_increment_age`39行；`ekf_localizer/src/ekf_module.cpp::measurementUpdatePose`约220行；`kalman_filter/src/time_delay_kalman_filter.cpp` |
| 生产R缩放/五次倍率 | `ekf_localizer/src/ekf_localizer.cpp` NDT callback（与GPS/LIO callback分开）；`measurement.cpp`38–45行选XY/yaw并乘smoothing_step；config predict100/extend15/smoothing5/gate10000 |

公开官方源码可按固定commit核验：[FAR-LIO ef205103](https://github.com/TUMFTM/FAR-LIO/tree/ef20510316ad1f8dbf2e428e4a193f700493eccd)。受限生产源码只读审查并保留身份，不复制发布。源码能力不等于本次运行每项机制都触发，实际触发证据另见聚合及报告。
