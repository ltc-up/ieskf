# Phase AL conclusion — PARTIAL

Real-input shadow chain runs.284 unique historical Sources and matching timestamps were checked; fresh IMU/wheel local15D replay equals no-map control for all1470 state/P outputs bitwise. No map update enters local prediction, velocity,bias or P. Common historical fused initialization and uncalibrated model are explicit.

155 candidates shadow-accepted,129 NOT_CONVERGED;284 shadow publications can include held correction.0 trusted anchors. At accepted candidate time, composition max element error 3.41e-13. Duplicate and delayed tests report DUPLICATE/STALE. An injected wrong global Z candidate is shadow-accepted while local history is unchanged: isolation is achieved, correct global matching is NOT.

Core timing measured on this run: median0.0411ms,p950.0470ms,max0.0710ms. Excludes GPU/Deskew, ROS callback scheduling and input loading; not20Hz qualification.

ROS2 research node actually started in isolated domain174 and transported568 real-input-derived events, with284 correction publications and no TF. Typed custom-message package build outcome is in build_results.json. This is an offline transport test, not live production sensor qualification. Candidate metadata is richer than legacy pose messages; unknown support/covariance is preserved.

Continue automatically to AM. Real geometry diagnostics can proceed from stored Sources/map. New CUDA matching is subject to driver availability; statistical fusion stays blocked until qualified evidence. [audit](SOURCE_INTERFACE_AUDIT.md), [interfaces](INTERFACES.md), [failures](FAILURES.md), [results](results.json), [reproduce](REPRODUCE.md).
