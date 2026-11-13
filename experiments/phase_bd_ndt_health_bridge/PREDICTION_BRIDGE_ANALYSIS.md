# Prediction bridge and map correction

The real gap is wrong accepted map observations, not the absence of all motion prediction. Current ROS and research implementations already propagate without new NDT. Merely adding map/odom names does not create an independent odometry estimator: AY state p/R already lives in the map chart and has NDT corrections in mean/P.

With notation T_A_B mapping B coordinates into A, the research seed is T_map_base with translation p_IMU_map−R_IMU_map*lever_base_to_IMU. A genuinely independent T_odom_base would give T_map_odom=T_map_base_accepted*inverse(T_odom_base_at_same_Header). Compose at the same timestamp and preserve the IMU/base origin convention. Do not feed both this correction and the same NDT into the filter. No production TF architecture was changed.

## Actual delayed counterfactuals
Baseline and only-invalid branches accept all1468 original valid observations and are identical. Baseline native prior means/R/P match original records exactly. Arrival order, delayed insertion, sensor corrections, full15P,500ms eviction and suffix replay are executed; this is not removal of rows from a trajectory CSV. Subsequent NDT poses remain frozen, so this is a counterfactual of filter information flow, not new causal CUDA matching after rejection. Actual causal matching is evaluated separately in P0.

At preselected median frame of fixed sections3,9,5,8, reject30 consecutive original valid NDTs. During each rejection block478–480 actual IMU/wheel events continue in PREDICTION_ONLY mode.30 rejected scans cover1.5s; previous-to-next accepted observation gap is about1.55s. Transaction state REJECTED means no insertion; ACCEPTED means an actual new insertion; first resumed insertion is RECOVERING. Baseline has1468 accepted; each window has30 rejected,1437 ordinary accepted and1 recovering. Histories are never cleared.

|section|max lateral difference from all-NDT branch m|first recovery current-position correction m|
|---|---:|---:|
|3|0.168490|0.098922|
|9|0.352575|0.206822|
|5|0.629504|0.396536|
|8|0.223803|0.140568|

Differences are engineering counterfactual differences, not absolute drift truth. Max values are over retained subsequent replay, not claimed to occur exactly at rejection end. Recovery correction compares current mean immediately before/after delayed insertion at the same processing event; it is not total vehicle motion between scans. The frozen filter has no added five-step blend, so a nonzero immediate correction is real. Control-safe continuity is not proven. The73.35s all-NDT-rejected branch continues numerically but diverges by up to6741m laterally relative to baseline; this is a failed long bridge, retained in full, not an acceptable fallback.

The historical30-frame1.281m result is not renamed pure prediction: the production five-attempt queue could continue applying prior items. New15D cases have no such queue, retain actual wheel/IMU corrections and past posterior information, and cannot be transferred to a live ROS control guarantee. No automatic filter reset or high-speed global jump exists.
