# Measured performance and limits
All new timing is offline host wall time, not an online ROS2 executor measurement. `loop*_results.json`, `original*_results.json`, `global*_results.json`, `timing_results.json` and `local_timing_results.json` provide P50/P95/P99/max per actually measured operation. Complete input and failure coverage accompany these timings.

On whole loop44, existing 15D propagation P50/P95/P99/max is approximately 0.0696/0.0730/0.0801/6.99 ms; wheel update 0.0467/0.0496/0.0557/3.31 ms. The global qualification manager's event processing is approximately 0.00518/0.0308/0.0326/4.50 ms. These are separate distributions, not an end-to-end budget. Reproduction timing may vary with I/O and host load.

Local checkpoint replay on 43,441 real events achieves exact Header-reference final state/P with max buffer 153 events. Its actual per-ingest wall distribution includes suffix reprocessing; see local_timing_results. No IMU/measurement is fused twice into its own posterior. First failed implementation timing is preserved separately and not substituted for the corrected run.

Deskew, CUDA-NDT, per-source preprocessing, fresh multi-start diagnostics, GNSS coordinate anchoring and ROS2 queue/executor overhead were **not measured in a newly connected full-lap chain**. Existing AM CUDA numbers are historical, not AO end-to-end runtime. No sum of module P95 values is presented as end-to-end P95.

20 Hz complete matching budget: NOT_VALIDATED. 100 Hz live state publication/backlog: NOT_VALIDATED. Offline sensor-seconds/wall-second does not establish either deadline. ROS2 research nodes were not started for this AO causal chain. Required next measurement is on the isolated full causal stack with callback receipt timestamps, queue age/drop accounting and per-frame source identity; it must include cold-start/map-update costs and failures.
