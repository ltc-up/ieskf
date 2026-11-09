# AV — Prioritized repair evidence

Baseline c647ac3224adee775d30dc6f9553c9b83612c060. Original AR/AS/AT/AU inputs and outputs are immutable. AU identifies about10% unit lateral gain, but does not identify physically miscalibrated P versus R; no gain formula bug was found.

## Highest-evidence actionable defect: mismatched information-use semantics

Read-only production chain: `lidar_ndt_v1/src/ndt_component_node.cpp` publishes score-derived per-packet engineering covariance; `ekf_localizer.cpp::callbackPoseNDT` applies existing branch mappings; `measurement.cpp::poseMeasurementCovariance` multiplies by `pose_smoothing_steps`. `aged_object_queue.hpp::pop_increment_age` returns and reinserts the **same observation** until the configured age is reached. `ekf_localizer.cpp` uses this aged pose queue; checked-in smoothing_steps=5. These are queue-information compensation, not five independent point clouds.

Research `ndt_pose_observation.py::engineering_R` retains this factor5, but `PoseDelayReplay` accepts each cloud correlation group once. Delay suffix replay replaces the posterior; it is not five independent measurement applications. Thus the research single-packet update carries only one fifth of the historical packet's intended information budget. This explicit source-level contract mismatch is stronger evidence than choosing an R multiplier from GNSS error. The underlying packet score-to-noise mapping is still an uncalibrated heuristic.

For a fixed linear likelihood, N updates each with covariance N*R_packet have total information HᵀR_packet⁻¹H, equal to one R_packet update. A single N*R_packet update does not. Dynamic/nonlinear queue smoothing is not exactly equivalent to one update, but its compensation factor cannot be transferred without its repeated-use mechanism. Candidate removes that **absent-queue compensation**, rather than selecting a scale or claiming calibrated R.

## Other candidates not selected initially

P/noise calibration and time-correlated wheel/NDT errors remain unqualified; there is no sufficient evidence to insert arbitrary P inflation, a new slip time constant, zero-lateral constraint, Bias clamp or a tuned process noise. Native R did not improve AS. Independent GNSS timing/installation is absent, so it cannot select an R scale, source bias or sensor delay. Full H/attitude/reset/covariance are retained unchanged.

This is a verified information-accounting mismatch and plausible contributor to lag; it is not a unique physical explanation of every right-corner reference discrepancy. Full causal rematching, integrity checks and held-out comparisons must determine whether the repair is useful or instead exposes covariance/association failures.

## Actual environment recovery

A fresh original-policy40-frame raw three-LiDAR/own-motion Deskew/CUDA-NDT run completed. All40 seeds, final CUDA poses and full15D deltas match AR exactly (max differences0). Original GPU services/Target persist, target built once and startup reported separately. This confirms a usable execution baseline before changing the estimator interface.
