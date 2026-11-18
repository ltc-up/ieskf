# A2 frozen-prior Score MAP

The implemented objective is `F(z)=0.5 zᵀ(P−)⁻¹z−β S(q(inject(x−,z)))`. The 15D chart order is position, velocity, right rotation, gyro bias, accel bias. The IMU-to-base lever arm is applied by `p_base=p_imu−R lever`. The NDT chart is translation followed by RxRyRz Euler angles, not a rotation vector or ZYX convention.

`g=(P−)⁻¹z−βJᵀgS`, `H=(P−)⁻¹−β[JᵀHSJ+Σ_i gS_i q_i'']`. The pure coordinate derivatives are finite differenced; Score and its derivatives come from the real CUDA interface. Synthetic full-15D finite differences verify the complete chain rule. Prior remains frozen throughout both refinements. The Align-pose start fills velocity/bias with the conditional minimum of the same full prior, not a pose observation update.

Both prior and unmodified Align starts are refined for at most three iterations. Newton/SVD direction uses a sign flip if needed, a maximum actual mixed 6D pose step of 0.2, and GLL3 total-objective Armijo/backtracking. A failed minimum-step test rejects the step; it cannot accept a Score-only decrease. The four candidates are compared under the same frozen prior, beta and Score. Existing attitude/bias domain limits apply; no Fitness threshold or reference chooses a branch. All candidate objectives, z, q, original and approved H, trial acceptance, and selected identity are exposed for logging.

Conditional covariance is a separate explicitly uncalibrated model described in `BETA_AND_CONDITIONAL_P_PROPOSAL.md`. No LM Hessian inverse is used. One successful transaction commits one mean/full-P; retained replay starts before that transaction and recomputes from the actual preceding checkpoint. `ManagedReplay` owns terminal IDs and information-group tombstones; the historical C5 LegacyComparisonReplay itself does not provide those guarantees. An unchanged 100Hz snapshot publisher can observe committed states without entering this geometry solver, but actual state age and CPU blocking remain unmeasured.

The user has now frozen β=1 and approved the explicit continuous-path snap-domain guard described in `A2_FINAL_FROZEN_CONTRACT.md`. Fixed-association smooth-region derivative validation does not establish validity at voxel/gate discontinuities.

## Integration and statistical limits

The exact prior identity hashes floating state/P bytes and the integer epoch nanoseconds separately; a 1 ns timestamp change has an explicit regression test. The Source and Target recipe remain immutable across retained historical callbacks. Original Align availability gates only its own candidates. A legal Prior branch may still update when Align is invalid; no legal candidate leaves prediction and full P unchanged.

Conditional information uses pre-frozen D=(0.5 m,0.5 m,0.5 m,3 deg,3 deg,3 deg), dimensionless positive-curvature eigenvalues clipped to [0,1], and zero information for nonpositive/unobserved directions. This operation constructs conditional P only; it does not replace Score curvature in the MAP mean equation. Original and approved Hessians remain separate evidence. Neither the cap nor full-P SPD is statistical calibration. Late history changes must be audited through event lifecycle states; initial acceptance alone does not certify final retained geometry.

## Complete-run lifecycle audit

Both full-map A2 runs completed without prior-hash or full-P numerical anomalies. On 1013 all 7798 observations have one retained semantic revision; 7796 updates are conditional and two invalid-domain observations preserve the prior mean and P. Raw-Prior selection still performs one conditional covariance update (552 frames), and must not be mislabeled prediction-only. Complete results do not certify statistical consistency, convergence or correct-basin identification.
