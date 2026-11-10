# Actual offline performance, not ROS realtime

A1468 processing section P50/P95/P99/max26.44/36.96/41.47/137.72ms. A4134:27.23/38.10/47.27/109.67ms. Initial Target/covariance preparation≈0.80–0.81s is separated. Three-lap modeled Header-to-complete P95/P99/max96.80/153.49/306.87ms; no500ms history loss. This measured offline causal scheduler includes recorded arrival age and actual processing times, not a fabricated completion clock.

Instrumented1468 service transactions (9342 calls): wallP50/P95/P99/max1.169/5.721/6.910/8.134ms; service internal0.979/5.455/6.630/7.852ms. These mix correspondence refresh/Source preparation and frozen residual evaluation. Do not add theirP95 to whole-frameP95. Source covariance is prepared once per actual frame; target tree/normal data once per process. Each iteration accumulates small matrices; no dense point covariance. RPC/JSON and state replay remain research overhead.

B failed-prefix sectionP95≈26.43ms,max75.20ms. It is not a full-lap/three-lap latency qualification. Native Gaussian export and GPU build are startup operations; B residual evaluation is CPU. No new CUDA residual kernel exists.

Both branches retain CUDA Deskew. No production ROS executor, live queues,100Hz publisher, GPU utilization/peak-memory campaign or end-to-end sensor callback test was performed. P95 fits the nominal50ms budget on this host; maxima violate a strict deadline. Offline20Hz deadline **FAILED**; ROS20Hz/100Hz **NOT_VALIDATED**; production **NO-GO**. No buffer expansion, synthetic clock normalization, slow-frame dropping or per-lap reset masks these results.

Actual instrumented1468 moduleP95: CUDA Deskew1.603ms, crop/voxel2.456ms, rebase/file write2.167ms, direct-MAP observer12.639ms. AllP50/P95/P99/max are in module_timing.json. Observer includes its residual RPCs but excludes suffix repropagation. Separate IMU/wheel/replay per-call timings were not instrumented in AX; the full measured section includes them. Do not infer a missing module time by subtracting quantiles.
