# Phase AN conclusion — PARTIAL implementation / NO-GO qualified real fusion

Implemented runnable source health,global coordinate anchoring,dimension expiry,mode management and loss/recovery without a local-state/P write path.10 new tests and118 total pass. Synthetic qualified-position fixture exercises recovery while preserving unobserved yaw; it is separate from actual GNSS data.

Actual same-window run processed284 NDT groups and146 raw GNSS records. Two real multistart ambiguities are retained. No real GNSS observation meets map/antenna/time/protocol/statistical prerequisites; no NDT covariance was invented.0 qualified anchors,0 global correction publications,0 trusted global availability. Qualified real recovery and meaningful full controlled A/B/C improvement therefore FAIL. Raw source data are present; this is qualification failure,not missing GNSS/clouds.

Local state/P stays equal to no-map control; wheel-induced Z4.177m and vertical uncertainty growth remain. Research ROS2 manager actually transported3654 events with1470 matching status outputs and no TF. This validates isolated interface execution/abstention,not production or20Hz.

Use the implementation to expose information gaps and ambiguity; do not deploy reject-all behavior as localization. Next work is qualified GNSS/map/reference and local vertical observation evidence plus structured current NDT publication export. No original point-plane Geometry-IESKF is needed by these modules. [Cross-stage summary](../PHASE_AL_AN_FINAL_SUMMARY.md),[comparison](CONTROLLED_COMPARISON.md),[GNSS audit](GNSS_AUDIT.md),[uncertainty](GLOBAL_MODEL_AND_UNCERTAINTY.md),[failures](FAILURES.md),[reproduce](REPRODUCE.md).
