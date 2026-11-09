# Failures and uncompleted qualifications

1. Initial direct antenna lookup in the historical motion export failed `STATIC_TRANSFORM_UNAVAILABLE`. Original bag TF extraction confirms antenna links are absent there too. Binding now explicitly uses a historical URDF candidate for missing links; it does not pretend they were recorded or calibrated. The failed assumption is preserved here; no estimator run was lost.
2. Current URDF has only a visual mesh, not a measured collision/body/tyre model. Historical copy supplies an active visual box for conditional polygon computation only. Actual tyre, wheel contact, steering-dependent front tyre envelope and physical safety validation were NOT_EXECUTED.
3. Independent75ms latency calibration, antenna phase-centre survey, map0105 height datum and historical boundary survey/runtime environment remain unverified. Accuracy ranking is delay-sensitive. No selected optimum was called calibration.
4. Initial planar box diagnostics were superseded by all-eight-vertex 3D silhouette computation. This changed old/GNSS minimum modeled margin from .150/.634m to .139/.619m. The preliminary values are preserved in this failure record; final private polygon records and public statistics use the complete projection. Neither is actual tyre geometry.
5. No independent wrong-peak labels exist. No detection rate, true RMSE, correct-peak assertion, or new safety GO is reported. No unnecessary GPU multistart or estimator rerun was used to manufacture evidence.
6. Complete prepropagation mean-state snapshots needed to attribute accumulated prediction discrepancy to a specific upstream sensor term are not available in every saved transaction. Current prior/NDT/posterior decomposition is exact; physical causal attribution remains partial.

Final directed/full regression:171 passed,0 failed. Missing qualifications are not counted as passed tests.
