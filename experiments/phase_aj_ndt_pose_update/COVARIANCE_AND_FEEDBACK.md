# Covariance, engineering rejection and feedback

C uses the entire15D predicted covariance, a3x15 observation Jacobian and Joseph update with a single right-chart reset. Direct Z, vz, roll, pitch and bias measurements are not inserted. Indirect changes remain. There is no point-plane GN/IRLS result, NDT Hessian or mixture/surface information matrix in C.

All66 submitted states/P are reproducible from their saved priors and measurement exactly to roundoff. All218 rejects preserve the paired prior. All recorded P matrices are symmetric/positive definite. A finite SPD matrix is numerical eligibility only; empirical fitness weights, shared Deskew IMU/wheel inputs, map error and temporal correlation preclude statistical calibration claims.

C final vertical covariance: P_ZZ=2265.3371m²; P_vzvz=35.90573(m/s)²; P_vzZ=273.13722m²/s; Corr(Z,vz)=.957707. The first partial geometry-induced Z collapse is absent; instead substantial vertical uncertainty and cross-correlation persist. The unchanged wheel update can then generate metre-scale Z corrections. No cross terms are deleted or manually bounded.

C's maximum actual LiDAR gyro-bias/accel-bias correction norms are .003158rad/s/.085703m/s² and roll/pitch error-subvector correction norm .024834rad. Actual bias-state norms and output increment distributions are in key_results.json. No velocity clipping, fixedZ, new noise scaling or stronger statistical gate is applied.

155 NDT outputs carry OBJECTIVE_CONVERGED;129 hit MAX_ITERATIONS. Some converged fits are poor: converged-fitness P95 is7.951 and maximum23.565. Two exceed the legacy R admissibility range. Large proposed updates are held by unchanged engineering checks, but five high-fitness events still submit with very weak legacy weights. Without independent pose reference we cannot certify a wrong local peak, or rule one out. Solver labels and small innovations alone do not prove geometric correctness.

This closed loop is causal: every NDT seed equals C's current prediction at backend precision; active-region motion is C-driven; no A/B future state enters. The altered observation affects future nominal/P and therefore future NDT basins and queries. Degraded matching, unobserved height, unsafe proposal sizes and sparse recovery coexist; their individual causal shares have not been isolated. No further tuning/replay was attempted.

Avoiding unqualified point-plane posterior submission is a valid structural difference from B, not proof that the tight-coupled Geometry-IESKF has been repaired. This is an explicitly loose NDT-pose engineering comparison on the same15D predictor.

## Direct evidence for lost retention through unobserved vertical coupling

Of87 engineering rejects,28 have planar proposal norm <=.5m and would satisfy the planar step length alone;66 have |dZ|>.5m (overlapping categories). The full3D position guard, not just planar pose mismatch, matters. At L079 immediately after the early acceptance run, NDT is OBJECTIVE_CONVERGED with fitness .109817. Innovation is only [−.03807m,−.09770m,+.005628rad]. Proposed XY is [−.003220m,−.008952m], but proposed Z is−.551458m and vz−.198604m/s. Original3D step safety rejects the entire transaction. Its signed Z contributions from X/Y/yaw are−.074410,−.487417,+.010369m; prior sigma Z is8.527m.

This is a concrete loss mechanism even without a direct Z observation and without requiring a demonstrably wrong NDT peak. It does not prove that the implied Z correction is physically correct or wrong; it proves why this actual Kalman proposal was rejected. Later NDT failures and weak fits coexist. Do not clip the vertical increment or delete its covariance crosses to make the planar observation pass.
