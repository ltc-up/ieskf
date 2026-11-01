# Phase AF predeclared plan

Baseline AE7c27abb verified against origin/main, clean tree. No AD objective/Hessian is used as observation information. Prediction state/P, IMU/wheel/Q/R, map, native Source, Deskew and original safety/trust limits are frozen. No continuous replay.

## Synthetic prerequisites (before real observation/update)
- Known horizontal infinite plane: surface-Jacobian rank3 with exact X/Y/Yaw nulls (spatially spread source). Two orthogonal/nonparallel planes rank5 (one intersection-line translation null); three independent normals rank6; multiple finite-support plane patches score ONLY plane distance, no edge/tangential residual.
- Exact Target aliases must yield identical surface geometry/registry. Exact Source copies preserve physical sample ID and are deduplicated, not counted as independent noise. Different independent samples can legitimately reduce epsilon noise, but shared plane error must cap certainty.
- Diagonal known synthetic plane-error law: tilt std0.01rad, offset std0.1m; epsilon std is unchanged research lidar_sigma0.2m. Also full correlated known law with Cholesky [[.01,0,0],[.002,.01,0],[.01,-.005,.1]] to test off-diagonal effects. These are GENERATIVE SYNTHETIC laws, not calibrated map covariance or an R tuning sweep.
- Split/copy one physical plane into1/2/4/8 aliases with the SAME parent latent error and reference frame. Compare naive independent patch errors as an explicit FAILING counterexample. Check anchor/covariance transport, full low-rank covariance, source/plane Jacobians, Joseph equivalence, PSD, reset and atomic rollback. Duplicate multiplicities1/2/8/32, unique-sample densities9/25/81/289/1089.
- Seeded Monte Carlo5000 samples checks the correlated generative R_eff, no parameter fitting.
- Stop real update analysis if observability/dedup/shared-error tests fail; do not alter limits to pass.

## Minimal real surface extraction, frozen rule
- Exact-coordinate deduplicate map records; same coordinates remain unmodified. Fit ONE TLS plane per3D grid cell, baseline existing source_block_m=5m, diagnostic2.5/10m; >=existing normal_neighbors20 UNIQUE points, original planarity_ratio0.05 and second_eigenvalue0.02. Fit all records of visited cells, not one normal per Target.
- Visit cells containing map centers within original1m radius of Source at prediction/G1/G2/AA, with one-cell halo. Cell universe is outcome-independent once those four frozen candidates are given. No source or normal re-deskew.
- Neighboring cells are grouped deterministically when normals within atan(sqrt(existing_planarity_ratio)) and mutual center-plane distances <=existing Huber0.3m. Whole-component TLS must pass same planarity/eigenvalue predicates, plus member center/normal agreement under SAME gates. Failed components are unresolved and EXCLUDED, never silently treated as independent planes. The group is a geometric shared-surface hypothesis, not ground-truthed physical identity.
- Group reference anchor is projected UNWEIGHTED cell-center mean; frame covariance is explicit. Record supports, fit scatter, plane parameters and patch parent IDs privately. Duplicate map extraction must be identical. Tile-scale changes are diagnostics, not parameter selection.
- Associate each Source to nearest accepted supporting map record within SAME original radius, ONE surface group per Source. Residual is n dot(y-c), with no derivative/information from support boundaries or target spacing. Frozen prediction association and each candidate's own association are reported SEPARATELY.

## Frozen comparison and update
Use only prediction, stored AD G1/G2, and AA's actual first accepted partial proposal. Never use AD energy for scoring or R. Surface data overlap candidate-generation data, so no independent accuracy winner.

Real plane covariance cannot be calibrated from one PCD. Show pure geometry/spectrum plus ONE explicit model-conditioned numerical scenario reusing the full correlated synthetic law above; label it TRANSFERRED_SYNTHETIC_SCENARIO_NOT_MAP_CALIBRATION. Compare shared versus erroneously diagonalized effective R only as diagnostic, not a tuned backend. Full15 prediction kept intact. Single frozen linear-Gaussian update/once-only error reset, NOT an iterated registration solution. Require original cumulative AND step trust guards; reject means both state and P retained. No clipping, covariance scaling or eigenvalue floors. All real transactions are discarded diagnostics, never carried to another frame.

Numerical posterior validity is separate from physical identity/covariance calibration. Unresolved same-plane grouping or map/source correlation means no continuous qualification. Full raw map/Source/states/P/planes/factors stay ignored; public independent code, synthetic tests and deidentified relative/spectral results only.
