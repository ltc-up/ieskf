# Actual source and sensor contracts

Production read-only HEAD aa50e35467f89b4963f882d1f5abaace46d0ce70; frozen15D c647ac3; AY364d246. Untracked production vendor and all historical stashes are untouched. No production source is copied here.

## Production is not the15D research backend

`ekf_converter/src/ekf_converter.cpp`, velocity selection around451–500: if polynomial-filtered Kistler x<1m/s, publishes wheel x and y/z0; otherwise filtered raw optical x/y and z0. This is **selection**, not independent parallel velocity fusion. Status logic uses stst0 as good, with wheel≤7m/s fallback tolerance; failed status/high-wheel-speed branches set configured forward/lateral variances50 rather than0.01. A greater-than5m/s wheel-versus-zero-resultant check avoids feeding stuck-zero optical to its prefilter. It is not a universal “all high-speed wheels are invalid” rule. No new1m/s claim is inferred from FAR-LIO.

`kistler_callback` reads velocity_raw, not internal IMU-corrected velocity. `velocity_sensor_filter.hpp` extrapolates a three-state velocity/rate/acceleration polynomial, reports uninitialized after1s. The Q expression contains integer divisions and nonuniform powers; copying it as a calibrated physical stochastic model is unwarranted. This study does not change it, fit its delay using GNSS, or claim it is equivalent to raw optical measurements.

Crucially, `ekf_localizer/src/ekf_module.cpp::measurementUpdateTwist` sets dim_y=2 and y=[linear.x,angular.z], selected delayed state vx/wz,2x6 H. `EKFLocalizer::timerCallback` calls this method. Published lateral/vertical zeros are not additional measured vy/vz updates in this backend. It is a6-state planar model, with lateral kinematics implied by its motion model, unlike a full15D body/world inertial filter. Do not confuse publishing rawXY with fusing both dimensions. Its low-speed observable-velocity callback also inflates longitudinal R to10000; smoothing/extended-delay processing differs from research checkpoint repropagation.

`wheel_velocity.cpp` in the current checkout selects front-left angular rate with radius/steering/yaw corrections, publishes base_link twist using node time; it does not independently measure all3 velocity components. AQ's prior description of a rear-wheel producer does not match this currently checked-out source. The historical loop44 bag is not assumed built from this HEAD. Keep its verified recorded scalar eav24-compatible wheel topic unchanged; four raw channels are diagnostics only. Package/topic strings containing eav25 do not establish a different wheel protocol. No new loop8 evidence is used.

`lever_arm_compensation.cpp` rotates raw optical and compensates angular lever. AY's full installed sensor transform and IMU-to-sensor offset are retained. Physical antenna/GNSS calibration is irrelevant to runtime; optical installation/spot reference and response delay remain uncalibrated. There is no new sensor noise certificate: producer0.01 and research0.26(m/s)^2 are configured engineering values.

## FAR-LIO verified source reuse

Local official Git checkout HEAD ef20510316ad1f8dbf2e428e4a193f700493eccd matches AK audit. Relevant functions are `node.hpp::measurement_callback/measurement_timeout_callback`, `state_machine.hpp::get_valid_fusion_vec`, `vehicle_model_handler.hpp::input_wheel_angular_velocities/input_steering_angle/get_status`, and kalman_filter/base.hpp covariance transition handling. They manage named typed velocity sources, fresh flags/status, watchdog invalidation and recovery. NonHolonomic explicitly supplies model-based zero lateral/vertical; SingleTrack derives sideslip from steering/wheelbase. These are not measured optical lateral velocity and are not transplanted. Supplied5/9-state EKFs do not implement our15D bias/SO3/Joseph model.

AZ independently implements source identities, velocity-dimension choice, timeout, repeated-good reentry and diagnostics; it preserves the original15D covariance and actual delayed time. No FAR source copied. We use a100ms qualification dwell instead of pretending its covariance decay calibrates our R. No universal low-speed-wheel/high-speed-optical rule is attributed to FAR-LIO.

## Actual loop44 sensor evidence

The frozen AY extraction contains raw opticalXY/status plus corrected outputs for audit only; selected first packet per fixed10ms Header bin, past-recorded status required. No GNSS is read by extraction or candidate. Four-wheel diagnostic extraction decodes52158 actual messages with all4 fields finite and nonzero across the frozen driving window. Relative rear/front differences can arise from radii, turn geometry, timing and slip; no single-wheel truth/calibration claim follows.

`sensor_study.py` compares actual wheel/optical increments, age and source differences by predeclared acceleration±1m/s², yawrate0.1rad/s and speed20/50m/s groups. Frame compensation uses the preceding original baseline state (up to100ms old) and actual static TF/gyro; this is a conditional diagnostic, not independent truth. Neither label “slip” nor a fitted calibration is inferred solely from their disagreement. Fixed ±0/50/100ms derivative comparisons are not a uniquely identifiable physical delay; no lag is applied to the filter.

Complete aggregate values are in results/aggregate.json; private rows retain native timestamps and decoded payloads. No raw trajectory, model dimensions, installed TF values or restricted source is published.
