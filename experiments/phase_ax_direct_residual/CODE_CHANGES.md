# Changes, reasons and actual effects

|Original problem|Change and mathematical basis|Actual effect / limitation|
|---|---|---|
|Only final GICP pose exposed|AW-based direct_service calls GICPFactor at current transform, returns fresh residualGN H/g/cost/support|Actual1468/4134-frame own-state feedback; not finalH-as-measurement|
|Full distribution has tangent forces|Normal subspace selected from target geometry and fused metric; synthetic rank/FD checks|Correct ideal-plane nulls; discards nonplanar/tangential evidence; accuracy regresses|
|Independent points cause excessive information|Explicit per-cell latent correlated errors, Schur; exact-coordinate aliases canonicalized|Dense-reference identity passes; true plane identities/noise and arbitrary density invariance remain unqualified|
|Patch-only actual updates drive bias-domain rejection|Explicit scan-common nuisance with AW frozen envelope, nonlinear SE3 coordinates, joint prior/marginalization|40/1468/4134 functional completion; not a new calibrated noise model; weak effective gain remains|
|Repeated correspondence refresh changes objective|Two refreshes then true fixed-association inner solve/line search|Conditional convergence; no global association/correct-peak certificate|
|Iterating could reuse prior/information|Frozen prior chart, one joint solve, one reset covariance commit; atomic rollback|Analytic/Joseph/delay tests pass; full15 crossP retained|
|Native Gaussian kernel hides raw surface geometry|Read original GPU SoA; algebraically recover only nonclamped raw moments, validate256 voxels|22,616 candidate planes; B still loses state after531 frames|
|Need exact causal/delay retention|Reuse AR equations and AW journals; add diagnostics only|Original40 and A1468 repeat exactly match; no production/default changes|

Original motion/Q/wheel/Bias/default config, native_pose_R, ExternalPose, production code and all historic transactions are byte unchanged. Explicit AX runner/config choices select the new observer. Rollback is the original AR runner or AW ndt_legacy. New common nuisance and local-map covariance are engineering hypotheses, not hidden P inflation or GNSS-optimizedR. They do not solve the general observation-noise problem.
