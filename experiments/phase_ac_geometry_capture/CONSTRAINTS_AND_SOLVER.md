# AC4 — guards and constrained local solves

Original global safety balls: position correction 2m, attitude-chart correction 0.15rad, gyro-Bias correction 0.03rad/s, acceleration-Bias correction 0.3m/s², all RELATIVE TO THE FROZEN prediction. Local step limits remain 0.5m and 0.05rad in the original chart semantics. No new velocity cap is introduced; velocity changes follow the full prior correlations. Units and state chart are unchanged.

Only existing constrained-reference machinery is reused: exact full15 robust objective, analytic constraint Jacobians, prior whitening, and the existing KKT diagnostic. This is not another 15D/6D equivalence or KKT proof campaign. SLSQP may evaluate infeasible trial coordinates internally, but accepted outer states must pass the original exact guards and trust limits. No state-component truncation is used.

| Association | Existing representative | Final status | Outer steps | Unconstrained GN norm | Active original guard |
|---|---|---|---:|---:|---|
| original | rejected_large_proposal | OUTER_BUDGET_EXHAUSTED | 60 | 4.09105 | gyro_bias |
| original | largest_committed_velocity_change | BOUNDARY_ASSOCIATION_FIXED_POINT_DIAGNOSTIC | 21 | 1.36886 | gyro_bias |
| original | well_converged_control | INTERIOR_ASSOCIATION_FIXED_POINT | 4 | 3.14249e-07 | none |
| nearest_valid | rejected_large_proposal | BOUNDARY_ASSOCIATION_FIXED_POINT_DIAGNOSTIC | 12 | 4.08671 | gyro_bias |
| nearest_valid | largest_committed_velocity_change | BOUNDARY_ASSOCIATION_FIXED_POINT_DIAGNOSTIC | 32 | 1.27041 | gyro_bias |
| nearest_valid | well_converged_control | INTERIOR_ASSOCIATION_FIXED_POINT | 5 | 7.74372e-08 | none |

The original first-frame nominal cycle is interior, so Bias/pose safety bounds are not its cause. The representative large-correction problems differ: with nearest-admissible selection they reach association-stable constrained boundary solutions with gyro-Bias active. They are legitimate LOCAL DIAGNOSTIC constraint solutions, not unconstrained stationary points. They are NOT submitted as Gaussian filter posteriors, and no unvalidated constrained posterior covariance rule is introduced. The well-converged representative remains an interior certificate under both rules; this one control does not establish absence of normal-scene regression.

Eight new capture-interface tests pass: trust Jacobian at a nonzero anchor; feasible prior-white bounded starts; symmetric/zero nominal starts; no-budget labels; stationary fixed-geometry certificate; low-support rejection; no prior/state/P mutation; infeasible-start rejection without clipping. Six nearest-admissible interface tests also pass: invalid-nearest recovery, original-radius retention, all-invalid behavior, Target weight conservation, shared-Target normal-fit parity and brute-force nearest-valid parity. These 14 NEW tests do not repeat AB's numerical proofs.


Publication boundary: independent research code and de-identified numerical aggregates only. Full state/P, map/Source points, event timestamps, Target identities, equipment/extrinsic information and original project source are not distributed. No independent pose accuracy, calibrated covariance or production qualification is inferred from local numerical results. See [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
