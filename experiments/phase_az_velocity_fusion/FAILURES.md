# Retained failures and limits

- Overall repair FAILED: M/S improve AY's normal8 reference difference but weaken right3 and worsen normal5; neither outperforms original and AY across the fixed requirements. No main/default promotion.
- X-only single-factor result does not show that opticalY is physically invalid. It removes both useful and potentially misleading information. S's rotation-invariant scalar speed still loses direction information and does not solve that tradeoff.
- The initial test incorrectly required lateral P to stay constant through IMU propagation. Real propagation increases P; corrected test checks nonzero lateral mean, one measured row and PSD, with independent model finite differences retained. Initial failed log `numerical_initial.log` preserved.
- Final code review also found that an exactly zero predicted planar speed could raise during health bookkeeping before low-speed fallback. The manager now handles this undefined norm Jacobian without submitting a measurement, with dedicated zero-speed and malformed-input tests. Recorded nonzero driving outputs remain bit-exact in final40.
- First M40 startup failed before processing due to duplicate Python import alias in text adapter. Fixed via explicit namespace class selection; failed0-frame directory/log preserved alongside M40_fixed and final S40 parity. It was not a matcher-convergence failure.
- Initial raw-wheel extraction dropped the sourced ROS PYTHONPATH, causing ModuleNotFoundError. Rerun retains ROS environment; no fabricated raw-wheel record.
- A read-only sandbox write of final40 parity initially failed after printing exact comparison. Repeated under authorized write execution; no data were lost or inferred as saved.
- Fixed sensor-only lag scan cannot uniquely identify physical delay; wheel slip, frontend response, optical noise and body dynamics remain confounded. No lag/lever/scale fitted against GNSS or sensor differences.
- Four decoded wheel channels are diagnostics; historical binary/protocol/tyre geometry is not reconstructed just because current source selects FL. Recorded trusted loop44 scalar wheel remains unchanged.
- Status+gross-jump guard catches the specified injection, not all valid-status errors. Slowly varying bias under5m/s, poor optical ground texture, wrong radius and simultaneous LiDAR failure are not certified. Source fallback succeeded with healthy NDT; no physical safety GO.
- Engineering opticalR/wheel slip envelopes and sample-dependence are uncalibrated. Selection avoids independent double-forward assimilation but does not eliminate sensor/state/map correlations. No calibrated NEES/NIS claim.
- GNSS physical time, reference installation and map datum remain conditional. At150ms reference the original has lower overall reference RMSE. No certified absolute accuracy, tyre-line safety or production claim.
- One profiled lap below50ms is insufficient: three-lap processing has181–205ms long tails. ROS2 live20Hz/100Hz output, actual queues/CPU utilization and physical observation-age guarantees remain unvalidated.
- Historical GNSS CSV original path is unavailable, as already recorded in AY frozen verification. Actual fixed reference-cache identity is hashed; missing original is not called reverified.

No old experiment, stash, production code or default was overwritten. All raw data, state/P, Sources, NDT results, logs and failures remain private; public figures/statistics are aggregates only.
