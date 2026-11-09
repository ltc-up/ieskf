# AT3 — Fixed right sections3/9 and control1

**VERIFIED replay decomposition; CONDITIONAL reference comparison; physical root cause remains unresolved.** IDs and reference tangents are frozen from AS, not chosen after AT results. Entry/middle/exit use the saved section progress range thirds. Native scan-end times are used for prediction, NDT and posterior, with oldEKF/GNSS interpolated to the same times. These values differ slightly from AS published-state-time metrics; neither is substituted silently.

Posterior reconstruction uses `p+=delta_p`, `R_post=R_prior Exp(delta_theta)` and the same IMU→rear-axle translation on each respective orientation. Across1468 updates `delta=K*innovation` maximum component discrepancy is5.56e−17. Full P and all cross terms remain intact; private causal.npz retains prior P, R, K, innovation and delta.

Absolute lateral GNSS-reference P95 (m):

| Fixed section | Valid frames | Prior | NDT | Posterior | OldEKF |
|---|---:|---:|---:|---:|---:|
|1 control|48|.352|.228|.337|.796|
|3 right|89|.721|.286|.674|.420|
|9 right|29|.686|.437|.668|.270|

The large discrepancy is already present **before the current NDT update**. NDT is closer to the engineering reference at the upper-tail statistic, and the posterior reduces the prior P95. Updates increase absolute reference discrepancy in17/48 control,12/89 section3 and3/29 section9 samples; all outcomes are retained. This does not prove every observation correct or a causal sensor defect. It contradicts attributing these entire corners to the current NDT update pulling toward a wrong peak.

Full-gain decomposition separates position-residual versus attitude-residual contributions to IMU-origin lateral correction; lever rotation is retained in the exact posterior. Section3 mean contributions are approximately−.0158/−.00067 m; section9 −.0120/−.00052 m. These are correction components, not fitted weights. Gyro/accel bias update norms and position/attitude cross-covariance norms are recorded in statistics.json. Source prior does not persist all intermediate preprediction mean velocity/Bias states, so an exact counterfactual attribution to a particular upstream IMU/wheel/Bias term is NOT_VALIDATED.

Yaw/innovation/Bias plots and AS source geometry are joined by exact native Header identity:48/89/29 matches. Full local rank, support, normalized vehicle lateral/longitudinal information and fitness are descriptive, not covariance calibration or correct-peak labels. The directional convention remains AS's map translation plus rotation scaled by10m, expressed along vehicle directions with coupling retained. No new R is derived. A yaw/lateral temporal association is not independently established yaw causation.

Time hypotheses can reverse global accuracy ordering; documented lever-chain reproduction rules out an accidental omission/inversion in the historical script, but actual time latency, antenna installation and reference orientation are still conditional. Independent correct-peak labels are unavailable. No frozen multistart was run: current evidence does not establish a specific wrong-peak frame, and the task permits that experiment only after identifying one. Competing peaks and physical NDT correctness remain UNKNOWN. Neither weak geometry nor a wrong peak is asserted as the root cause without evidence.

Most useful next investigation is independently measured GNSS age/antenna survey and persistent pre/post propagation state snapshots around the fixed right corners. This phase changes no filter, Q/R, bias model, matching parameter or time used in localization.
