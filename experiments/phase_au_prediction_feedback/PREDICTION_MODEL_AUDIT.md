# AU1 — Actual propagation and history recovery

**VERIFIED implementation and frozen-run equivalence; physical sensor/noise calibration remains conditional.** Baseline ac8e87f. No production, estimator, Q/R, extrinsic or NDT configuration changes; GNSS is not a filter input.

## Actual source semantics

`geometry_ieskf/state.py`: nominal state `[p,v,R,bg,ba]`; error order `[dp,dv,dtheta,dbg,dba]`. p/v are IMU-origin map quantities, R is map←base-axis rotation; bias coordinates are those body axes. Injection is additive except `R_new=R Exp(dtheta)`; reset rotational block is right-Jacobian Jr(dtheta). `external_pose.py` observes rear-axis base pose `p−R*lever`, with `H_position=[I,0,R[lever]x,0,0]`. Full SO(3) innovation is `Log(R_prior^T R_NDT)` with an identity local attitude H at the chosen prior anchor.

`inertial.py`: each Header event advances using the **previous held IMU**; there is no independent preintegrator. Midpoint R handles specific force; world acceleration is `R Exp((omega−bg)dt/2)(f−ba)+g`. Position uses v*dt+.5*a*dt²; velocity a*dt; attitude right exponential. Bias nominal means are held during prediction and changed by accepted wheel/NDT updates. Analytic F includes attitude/bias cross-couplings; P propagates FPFᵀ+GQGᵀ, symmetrized. Existing AQ finite-difference tests are retained. Sampled-ZOH noise is not exact continuous white-noise integration; PSD does not prove calibration.

`wheel.py`: scalar base-forward speed `e_xᵀ[Rᵀv−(omega−bg)×lever]`. No measured lateral/vertical velocity is invented. H has velocity, attitude and gyro-bias terms; all15D gain rows and all P cross terms remain. Actual eav24 loop44 input is used, not loop8. Kistler is not simultaneously fused: AR uses the exported wheel scalar and Bosch. Existing production source audit supports g→SI and angular conversion, runtime axis/calibration conditions, Kistler frame and source-selection differences; those physical qualifications are not redefined by successful replay.

`run_closed_loop.py`: recorded base pose initializes p/R once; initial v uses wheel+angular lever velocity; initial bias means are zero with the frozen configured P. Recorded base←Bosch rotation transforms raw gyro/force to state axes; its translation supplies the lever. No later historical production pose or GNSS is fed back. Initial roll/pitch/bias/noise validity is still not independently calibrated.

## Delay and execution semantics

`DelayedLocalReplay`, `PoseDelayReplay`, `AuditedPoseReplay`: Header-key order IMU→wheel→clock→NDT, bounded .5s checkpoint history; delayed NDT replaces the applicable pre-observation suffix and propagates it again. Re-execution is not a second independent fusion. Wheel/clock/NDT event times split held-IMU intervals. Arrival/complete scheduling in AR is offline processing, not real ROS executor timing; sensor recorder time is separately retained.

AR does not retain the precise CPU boundary between advancing motion just before NDT ingestion and the final complete time. AU uses the saved available/complete envelope, advances to complete then inserts the same delayed observation. This alternative boundary is qualified **only because** all1468 priors, seeds, gain/delta, acceptance, published full mean/P and published time are exactly identical (max discrepancy0). It does not prove online latency equivalence. CUDA results/Source/Deskew are frozen authorized inputs, not rerun.

## Recovered data

Recorder wrappers call unchanged original predictor/wheel/pose methods and log pre/post full mean/P, held measurements/dt, F, H/K/residual/R and event identity. Envelope execution:78054 stage records including repropagation revisions. Final Header history:23535 propagation calls,7357 wheel calls and1468 NDT calls,25003 events. All canonical native priors exactly match AR. Both private histories are retained, along with per-event checkpoints and original publication comparisons.

Windows are fixed before mechanism analysis: AS sections1/3/9 plus1s padding on each side, clipped to available history. Counterfactual representatives use each section's median frame index, never best GNSS error. Existing AT reference is conditional; velocity/IMU state increments are not themselves localization error. Equivalence is a prerequisite for the subsequent decomposition, not an independent physical accuracy certificate.
