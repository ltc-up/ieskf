# FAR-LIO source audit

Official repository cloned by Git independently; main at `ef20510316ad1f8dbf2e428e4a193f700493eccd`, no tag pointing at HEAD. Audit date2026-11-08. No submodule payload or third-party production code is vendored. Audit covers source and supplied launch configuration, not a built/runtime qualification.

## 2D / 3D alternatives

[lib_cpp/state_estimation_cpp/state_estimation_constants/include/state_estimation_constants/EKF_2D.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/state_estimation_constants/include/state_estimation_constants/EKF_2D.hpp) — `EKF_2D enums`. 5 states: global x,y, Euler yaw, body vx,vy; inputs yaw rate,ax,ay. Alternative template specialization, no built-in parallel fusion.

## 3D state

[lib_cpp/state_estimation_cpp/state_estimation_constants/include/state_estimation_constants/EKF_3D.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/state_estimation_constants/include/state_estimation_constants/EKF_3D.hpp) — `EKF_3D enums`. 9 states: global xyz, Euler roll/pitch/yaw, body vx/vy/vz; six gyro/accel inputs. No bg/ba states.

## Filter dynamics

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf_3d.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf_3d.hpp) — `EKF<EKF_3D>::predict`. Body-velocity dynamics include angular-rate cross velocity and gravity; Euler-rate transform, forward integration and A/B Jacobians. Different from repository SO(3) right-error15D/world velocity/midpoint predictor.

## Components

[ros_packages/tam_state_estimation_node/components/state_estimation_node.cpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/ros_packages/tam_state_estimation_node/components/state_estimation_node.cpp) — `StateEstimation2DEKFKin / 3DEKFNh etc.`. Five registered choices, each owns one StateEstimation instance. docker-compose.yml selects the3D nonholonomic executable; no two-filter parallel runtime in that supplied deployment. Users could launch more instances, but that is not this architecture.

## IMU

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/imu_handler.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/imu_handler.hpp) — `update_input_vector / update_sensor_bias / set_sensor_bias`. Valid IMUs are averaged; stationary FIFO averages gyro bias with outlier conditions; accelerometer biases are restored to configured initial values. 3D subtracts handler bias. This is not full joint15D bias uncertainty or cross-covariance.

## Bias trigger

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/state_estimation.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/state_estimation.hpp) — `update_imu_offsets (around1298–1305)`. Enables handler bias update at absolute velocity<.4m/s. This threshold is source evidence only, not a target-vehicle stationary detector qualification.

## Vehicle models

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/vehicle_model_handler.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/vehicle_model_handler.hpp) — `input_wheel_angular_velocities / input_steering_angle / get_status`. Kinematic does not fuse wheel model. NonHolonomic averages radius-scaled wheel speeds into vx, sets vy/vz0. SingleTrack uses steering/wheelbase sideslip atan(tan(delta)*lr/(lf+lr)) to split speed into vx/vy, vz0. Tire-radius scaling interpolates velocity. Model status requires wheel OK and, for SingleTrack, steering OK. These implied zero components must not be transplanted without physical validation.

## Wheel channel

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/state_estimation.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/state_estimation.hpp) — `set_input_wheelspeeds / set_input_linear_velocity`. Vehicle odometry enters last velocity slot, including status. 3D velocity has hard component-squared-distance threshold and an outlier buffer, separate from EKF residual saturation.

## Registration

[ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/node.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/ros_packages/tam_state_estimation_node/include/tam_state_estimation_node/node.hpp) — `constructor / measurement_callback / measurement_status_callback / measurement_timeout_callback`. parse_sensor_configs builds named typed position/orientation/velocity channels with masks, indices,R_init,outlier bounds, subscriber and optional status; bounded static slot counts. Watchdog timeout invalidates source; step consumes fresh flags. Recovery requires fresh input and allowed source status, not automatic recreation of a sensor.

## State machine

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/state_machine.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/submodules/state_machine.hpp) — `set_position_status / set_orientation_status / update / get_valid_fusion_vec`. OK/WARN mark position/orientation valid; ERROR/STALE invalid. Aggregates localization, velocity, IMU and side-slip availability, with buffered error/stale paths. This is not a distinct NDT ambiguity/subspace state. StateEstimation::step also allows a low-XY-covariance safe override, not suitable as independent integrity evidence.

## Covariance adaptation

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/base.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/base.hpp) — `set_position_covariance / set_orientation_covariance / update_measurement_covariance_matrix`. Targets source-reported covariance diagonals, floors by configured R_init, changes R on valid-bit transitions and decays toward targets. Orientation and later slots use altered limit/decay. Does not learn NDT truth from fitness or speed; no full correlated6D statistical calibration.

## Update / rejection

[lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf.hpp](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/lib_cpp/state_estimation_cpp/tam_state_estimation/include/tam_state_estimation/kalman_filter/ekf.hpp) — `EKF::update`. K=Ppred H^T(HPpred H^T+R)^-1; x=xpred+K residual; all correlated states can move. P=(I-KH)Ppred. helper/outlier_rejection.hpp box clips residuals; named Mahalanobis function normalizes using configured bounds, not innovation S, with angular box clipping. Do not call this an atomic rejection or statistically validated robust covariance update.

## External pose boundary

[config/far-lio.yml](https://github.com/TUMFTM/FAR-LIO/blob/ef20510316ad1f8dbf2e428e4a193f700493eccd/config/far-lio.yml) — `measurement channel configuration`. LiDAR odometry arrives as nav_msgs/msg/Odometry with separate status; node active masks determine fused parts. No registration backend type enters EKF mathematics. External NDT could be adapted to this channel without GICP, after timestamp/frame/reference/covariance/health changes. No NDT integration or C++ build was executed in AK.

## Transfer decision

Reuse design patterns: channel identities, explicit validity/freshness, timeout callbacks, separate model handler, configurable measurement masks, gradual source reentry and diagnostic naming. Implement these independently with our richer ambiguity/accepted-subspace and correlation contract.

Adapt rather than copy: frame/reference/delay compensation, source covariance semantics, hard rejection transactions,15D bias handling, Joseph/reset covariance, partial-direction observations, shared-source accounting and target-specific wheel/Kistler physics. The wrapper extrapolates delayed measurements and inflates source covariance; this is not proof of correct delayed cross-correlation handling.

Cannot directly migrate:5/9-state template dynamics as15D ESKF, stationary gyro bias averaging as stochastic bg/ba estimation, implied wheel vy/vz0, scalar R adaptation as calibrated NDT noise, or residual clipping as correct outlier exclusion. FAR-LIO does not provide evidence that low speed/high speed alone should decide localization-source confidence.

Yes, the state estimation/multi-source design can be used independently of its GICP frontend. The existing generic input boundary supports this conclusion at source level; operational NDT adapter feasibility remains UNVERIFIED until integration tests. Retaining our15D motion backend and borrowing source management design is the recommended transfer.
