# AP performance
All AP timings are offline core event wall times. Nine whole-record schedules include history insertion, qualification, status and publication checks; each has91707 output slots. P50/P95/P99/max are recorded separately per scenario in `fault_results.json`. No complete causal ROS2 latency is measured, and these percentiles are not added to another module's percentiles.

Real frozen-geometry support diagnostics over the five fixed inputs take approximately6.05–7.82ms for the unchanged Target and5.90–7.31ms for the altered Target on this run. These include CPU support queries/normal analysis at a stored pose, not NDT registration, full preprocessing, GPU scheduling or an end-to-end50ms budget.

No fresh CUDA multi-start batch or raw partial-LiDAR frontend was executed in AP. Prior AM warm/cold GPU timings are historical and not a new performance result.20Hz matching capability and100Hz live ROS2 publication/backlog are NOT_VALIDATED. An offline fast all-rejection manager is not evidence of real-time useful localization.
