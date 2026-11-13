# Production and research pipeline audit

## Two implementations, not one fifteen-state production node
Production checkout HEAD aa50e35467f89b4963f882d1f5abaace46d0ce70 is on ndt_test, not a main checkout. Its tracked files were untouched; the pre-existing untracked fusion_vendor directory remains. The supplied c647ac3 is the independent research repository main baseline. The separate auto-eagle-main directory is not a Git checkout. Do not silently equate these identities.

Actual production state_index.hpp enumerates X,Y,YAW,YAWB,VX,WZ. This is a six-state planar augmented delayed filter with separate simple Z/Roll/Pitch filters. It predicts using its motion model and twist inputs. AY/AR is the independent fifteen-dimensional inertial-state implementation (p,v,R,bg,ba) with full cross covariance, IMU propagation and wheel/AY optical corrections. The statement that the inspected ROS node is a15D ESKF is contradicted by source. Both continue motion prediction without fresh NDT, but their queue mathematics differ.

## Actual ROS five-attempt queue
AgedObjectQueue pushes age0. Each timer tick captures initial queue size and visits each queued item once. pop_increment_age increments age and requeues while age<5, then returns the item for measurementUpdatePose. A failure inside the measurement update still consumes one attempt. Rejecting a new NDT before enqueue leaves old observations queued. Existing items may therefore affect up to four more ticks after their first attempt, typically40ms at100Hz. This is five uses of one observation, not five independent scans. Real-header C++ test passed this lifecycle; existing research replay tests also passed.

Pose covariance is multiplied by smoothing count in poseMeasurementCovariance. Callback first rejects covariance diagonals too small/large, then applies existing piecewise diagonal scaling; high Fitness warnings must not be confused with those covariance admission checks. The downstream legacy Mahalanobis calculation uses predicted pose with latest P subblock and threshold10000, not standard innovation covariance HPH'+R. It is not the NDT q50 point-Gaussian gate.

Current ROS YAML uses extend_state_step15 and predict_frequency100Hz: nominal finite history is on the order of140–150ms, not a universal500ms. Actual delay allowance comes from accumulated_delay_times_.back(), and out-of-history observations return false. updateWithDelay uses the augmented historical state/cross covariance. It is not the AY chronological event-repropagation implementation. Parameter overrides of a live deployment have not been measured; these are checked source/YAML values.

## Actual15D lifecycle
DelayedLocalReplay has horizon500000000ns, retained event keys/checkpoints/snapshots, explicit duplicate/history checks and ordered suffix replay. PoseDelayReplay adds independent NDT correlation-group deduplication. A late observation is inserted at its Header checkpoint, and later events are recomputed from the preceding checkpoint, not fused again into a posterior already containing that same measurement. Aging evicts events into the base checkpoint; their information is not erased from the state. Rejection does not clear history or reset mean/P. No ROS five-attempt smoothing queue exists in this path.

Audited counterfactual baseline replays all25003 original arrival-order ingests through the actual15D implementation;1468 native prior means/R/P reproduce with maximum difference0. Sensor advances and500ms retention remain active when NDT insertion is rejected. Previously accepted NDT information stays in the prior and may be recomputed during late-event revisions; it is not newly admitted rejected information.

## Observation cases
No new observation: keep existing prediction/queue lifecycle; no fabricated measurement. Nonconverged/nonfinite: preserve existing necessary rejection. Converged wrong branch: current necessary checks can accept it. Correct large correction: cannot reject solely from magnitude, score or legacy NIS. P0 and historical12632 establish that convergence and curvature do not resolve the last two cases. No Q/R, map, static TF, production defaults or active-region policy changed.
