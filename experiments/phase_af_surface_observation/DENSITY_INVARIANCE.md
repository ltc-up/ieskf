# Density and identity

Executed controls:

- Single known surface with Target copies: exact-coordinate dedup/physical registry yields identical normals, anchors and membership. Sparse3-point support repeated20 times cannot fake the required20 unique map supports.
- Real Target duplicated2x: extracted surfaces and prediction association exactly equal. No map overwrite.
- Synthetic Source aliases1/2/8/32x and real frozen Source aliases8x: identical canonical observations, information and posterior (difference norms0).
- Singular perfect-copy covariance A R A.T with pseudoinverse gives the same information as canonical observations. No jitter creates extra information.
- Independent new Source samples9/25/81/289/1089 legitimately reduce epsilon uncertainty but converge to the shared plane floor. This differs from aliases and from unknown correlated lidar returns.
- Known coplanar map tilings2.5/5/10m all produce one parent surface and preserve XY/Yaw nulls. Same-plane anchor covariance transport preserves effective R.
- The deliberately independent patch counterexample shrinks uncertainty without new geometry, retained in key_results.json.

Real diagnostic tilings are NOT pure density perturbations: coordinates remain identical but fits, supports, anchors and hypothesized plane identities change. Baseline5m:96 groups,1368 supported Source, conditional sigmaZ0.056174m. At2.5m:63 groups,479 Source,0.114965m. At10m:100 groups,1988 Source,0.062603m. These are uncalibrated model-conditional values at the SAME prediction; none is selected as best. Unresolved components0/4/1 for2.5/5/10m are excluded, not silently assigned independent noise.

In common map supports, six baseline groups split at2.5m; three split at10m, while17 coarse groups merge multiple baseline groups. These overlap counts are geometric, not physical identity truth. Reusing the same synthetic covariance at newly anchored/refit groups is a different conditional hypothesis, not coordinate-invariant calibration. Thus exact-duplicate invariance passes, but arbitrary real segmentation/density robustness is NOT established. A real caller must provide verified physical identities/covariance transforms; the API cannot infer them from labels.
