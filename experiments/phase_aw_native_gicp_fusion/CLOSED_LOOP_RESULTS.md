# Actual own-state GICP, complete lap and complete continuous three laps

A2ndt-free GICP loop actually reads original three raw LiDAR inputs, own IMU/wheel historical trajectory, persistent CUDA Deskew, own Source, own seed/ROI, CPU small_gicp, fixed6D engineeringR, complete15D update and suffix repropagation. Only initial state follows the same lawful frozen initialization; no subsequent production/other-branch state or future GNSS is fed back. Original state/P cross terms, SO3, lever and500ms cache remain.

|Actual run|Requested/processed|Updates|Rejected matching candidates|Longest update loss|
|---|---:|---:|---:|---:|
|GICP lap|1468/1468|1401|67|0.25003s|
|GICP continuous3laps|4134/4134|3956|178|0.25003s|
|NDT same fixedR control|1468/1468|1468|0|normal50ms cadence|

No per-lap reset, slowed input clock, extended retention, discarded slow frame or frozen NDT measurement injection. Actual matching backend is CPU small_gicp; CUDA is used for Deskew. Legacy`ndt` transaction key/rejection string are compatibility labels; annotated metadata fixes the inherited runner's misleading CUDA/Fitness descriptor while preserving raw summaries.

|Preselected lap|Frames|GICP updates|GNSS raw valid|GICP XY RMSE|original NDT XY RMSE m|
|---|---:|---:|---:|---:|---:|
|1|1468|1401|1204|0.596|0.578|
|2|1352|1293|1235|0.640|0.626|
|3|1314|1262|1196|0.715|0.501|


Per-lap engineering-reference results worsen relative to originalNDT, most strongly in third lap. Different1204 raw-mask versus1197 historical-mask first-lap RMSE values must not be mixed. Extended raw reference uses the same frozen AT math and explicit masks, not a newly fitted truth.

All saved P are symmetric and numericallypositive; complete15D deltas equal fullK×innovation. StateZ remains bounded over the observed map-height span; this is not a certified height or Bias accuracy claim. Source/matcher results change with own feedback; full private state/P/innovations/parameters/failed matches are retained. No wrong-peak correctness/detection accuracy is established by healthy numeric recursion.

Fresh legacy40 exactly reproduces AR. A separate fully timed1468-frame GICP replay agrees with the mainGICP lap to≈1.14e−13 pose,8.89e−16P and identical acceptance; this validates instrumentation/memory-management equivalence. The repeat is not a distinct held-out accuracy trial.
