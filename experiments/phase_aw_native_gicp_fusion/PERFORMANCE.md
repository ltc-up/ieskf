# Offline throughput measured; strict deadline/ROS qualification absent

Measured warm-start startup: GICP lap490.189ms,3lap538.986ms, including original Deskew context and static CPU target preparation; separate from per-frame processing.

Same-input AB align P95: CUDA-NDT2.820ms, small_gicp6.950ms. Continuous own-state GICP alignP958.357ms; prior loop7's18.79ms cannot be compared as the same workload. Current Source is only a few thousand voxel points and Target/preprocessing differ from those older experiments. No claim of CUDA GICP, GPU acceleration of GICP or algorithm score comparability.

|Component, actual continuous3lap samples|P50|P95|P99|max ms|
|---|---:|---:|---:|---:|
|gpu_deskew_ms|0.894|3.704|4.685|5.231|
|crop_voxel_sample_ms|1.242|2.458|2.698|3.636|
|rebase_with_file_write_ms|1.489|2.271|2.726|13.883|
|source_tree_ms|0.298|0.645|0.748|1.008|
|source_covariance_ms|0.977|1.531|1.727|1.935|
|align_wall_ms|2.640|8.357|13.597|19.022|
|information_refresh_ms|0.206|0.379|0.417|0.605|
|service_wall_ms|4.205|10.441|15.829|21.474|
|model_observation_age_ms|74.256|85.752|118.582|287.331|


GICP4134 processing-section wall P50/P95/P99/max17.740/26.966/37.187/109.653ms. Nominal50ms budget is usually met in thisoffline processing section, but strict full-run50ms deadline FAILS because of the tail. Max modeled Header-to-completion287.331ms;0 above500ms history. Raw arrival delay and actual computation/queue are different quantities. Model clock uses recorded availability plus actual elapsed processing; it is not a wall-clock ROS executor. Source input order and history horizon are unchanged.

|Inclusive timed full-lap operation|Calls|P50|P95|P99|max ms|
|---|---:|---:|---:|---:|---:|
|imu_predict|54453|0.073|0.100|0.170|2.043|
|wheel_correct|17533|0.046|0.056|0.063|0.427|
|external_pose_correct|1401|0.246|0.298|0.584|1.031|
|ingest_ndt|1401|2.506|3.043|3.845|4.640|


`ingest_ndt` includes external update and suffix replay, so it MUST NOT be added to `external_pose_correct` P95. IMU/wheel calls include replay repeats; call count is not distinct input count. Module percentiles cannot be summed into end-to-endP95. Whole processing-section timing excludes some final JSON materialization at end-of-run, and modeled completion can omit some serialization after its mark; no physical20Hz freshness/100Hz publisher guarantee follows.

Fully profiled actual1468 replay resource record:48.14s userCPU,3.17s systemCPU,30.34s elapsed,169% jobCPU,610020KiB time-reported maximumRSS. This is the reported process-family maximum, not summed simultaneous processes, device memory or portable CPU utilization. TBB arena4 is frozen. Static global target prep/tree/covariance happens once; per-frame Source prep, finalH refresh and own-state replay remain included. CUDA Deskew persists; GPU per-process peak/utilization was NOT_LOGGED and is not fabricated.

Runtime memory management streams JSONL, retains bounded Python records and freezes static input GC generation while GC stays enabled; it does not disableGC, fake completion or extend history. Legacy40 and complete profiling equivalence are verified. Actual4134 completion is a meaningful improvement over AV's growing-ledger runtime failures, not proof all-load real-time behavior. No controlled AV-V3 rerun with this memory change was performed; algorithm costs also differ, so current completion cannot prove GC/ledger retention was the unique cause or cure of AV failures. ROS20Hz/100Hz NOT_VALIDATED; production NO-GO. CPU GICP is not currently expensive enough, nor accurate enough, to justify a CUDA port as the priority.
