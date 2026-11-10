# Numerical and reproducibility validation

Final full suite: **195 passed,0 failed** (183 baseline plus12 new AY tests). Original183 tracked model/history files are byte-identical. `verify_frozen.py` checks actual motion/config/AR/AS transactions and available AT inputs against recorded SHA. Missing original GNSS CSV location is explicitly reported, not silently asserted unchanged; frozen evaluation-cache SHA is recorded separately.

New tests independently cover full15D optical sensor-axis/lever/right-error Jacobian finite differences; actual nonzero lateral velocity with no pseudoZ; full cross-covariance/Joseph/reset; atomic invalid status; rotational lever sign; delayed versus on-time update equality; duplicate event rejection; optical-forward substitution and timeout fallback; full16D mounting-yaw Jacobian and yaw gauge direction; constant calibration state/full cross propagation; invalid transforms/axes; independent16D Joseph/reset reconstruction. Final log is private `regression_final.log`.

The first16D process-noise mapping unit test failed; fixed to embed GQG.T and rerun before any real C execution. Failure log is retained. No test result is inferred from real-track smoothness.

Original mode actual CUDA40 frames reproduces seed/P/final NDT/K/delta exactly. XY cycle-profile full1468 also reproduces these exactly. These finite equivalence checks do not certify all environments, live ROS timing, statistical covariance or source independence. Real-loop positive P eigenvalues and observed bounded means are numerical evidence only.
