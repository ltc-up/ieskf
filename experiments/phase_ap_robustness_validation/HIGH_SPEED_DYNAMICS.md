# High-speed / dynamic evidence
Whole loop44 raw scalar-wheel channel P50/P95/P99/max is29.414/63.951/68.284/70.155m/s. Bosch angular-rate magnitude P50/P95/P99/max is0.05466/0.52204/0.71471/0.95113rad/s. These are raw channel distributions, not independently measured speed, yaw truth or a validated lateral-acceleration model.

The whole AO stream was processed without map feedback, with full correlated covariance and existing wheel rejection gates. Severe local vertical behavior persists. No Q/R or vehicle constraints were tuned in AP. Same-source GNSS does not certify high-speed accuracy; unknown timing/lever errors matter more at these speeds. Offline delay replay equivalence on the tested loop7 prefix is useful scheduling evidence but not full-speed ROS2 deadline or physical-model validation.

No new causal Source/Deskew/CUDA-NDT loop was run at high speed, so frontend health, multi-peak recovery and matcher initialization under dynamic motion remain NOT_VALIDATED. Raw high speed alone is not a successful robustness test. The correct qualification is NO-GO for stable high-speed localization under the tested uncalibrated local model.
