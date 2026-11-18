# KINEMATIC_BK actual wall-clock qualification harness

This is a separate observer harness; the frozen serial runtime is unchanged. No real-time experiment has yet executed. A syntactically valid harness and CPU transaction tests are not 50ms or 100Hz qualification.

## Ownership and causality

`kinematic_realtime_runner.py` consumes original raw CDR through the original recorder-order stream, released against a monotonic real wall epoch. Main ingests IMU/Wheel into unchanged A0 and independent pure Legacy replay. Main alone commits returned original Align poses through the original Legacy R once. It never invokes a GPU matcher. One worker owns original Deskew, global CUDA Align, local PREVIOUS matcher and its continuous O/history. Worker freezes historical A0 Deskew poses at dispatch, creates one Source, supplies the exact same Source to global and local branches, and counts Deskew + Align + Target + local match + commit in the cycle. It uses copied gyro/Wheel measurements and independent SE(3) intervals; only first-frame pure U supplies the approved fixed axis initialization.

`MotionStore` copies only already-released raw gyro/Wheel events under a short lock, with a bounded 1024-sample buffer. Publisher/worker prediction uses its own predictor instance. Therefore GPU work cannot occur through a main-thread `on_motion()->_drain()` callback. Main and publisher cannot update the worker's Ukin, H_init, O or Target state. A delayed worker interval may retry only after a subsequent actual raw input release. Complete header metadata can establish a true gap but cannot supply future measurement values.

The scan queue is limited to ten and main input delivery lateness over 100ms aborts explicitly. Historical Deskew no longer available within original rollback history also fails. The harness does not bypass such failures with the previously recorded serial trajectory. All failures retain a prefix and are qualification failures.

## Pose-only real 100Hz output

`kinematic_realtime_output.py` publishes ROS String packets and receives actual DDS loopback on localhost/domain173. Each valid packet contains local Pose, anchor time/revision, actual output time, source identity and original motion IDs; there is no 15D P, velocity or Bias joint-estimation claim. `predict_publication` computes actual SE(3) motion from the latest committed O anchor to actual publication time using released data and 25ms IMU/100ms Wheel held constraints. An invalid interval emits health-only, with no fabricated Pose or timestamp-renamed old estimate.

Output logs measure actual deadline lateness, copy/extrapolation, serialization, publish API, DDS loopback latency, receipt loss, anchor age and health-only counts. Publication throughput does not certify local accuracy or nonholonomic assumptions. A complete 20Hz period requires full worker-to-main completion including queue age, not merely local CUDA duration. Only isolated PREVIOUS is permitted in this harness; five-target serial comparison timing is retained separately.

## Initial checks and outstanding execution

Eight CPU tests cover exact curved SE(3) extrapolation instead of restamping, held-age rejection without Pose, immutable snapshot separation, future-available exclusion, monotonic commits/inputs, absence of GPU work on publisher thread pruning retaining held prefix, and an adversarial commit/prune interleaving that requires publisher revision retry. The publisher obtains a Pose revision, copies motion, rechecks the revision and retries boundedly if a commit intervened; repeated interference yields health-only instead of an incoherent Pose. The actual ROS/GPU run remains queued under root GPU ownership; no GPU work was launched by this agent. Full runtime identity hashes and source snapshots are saved by the new runner. The root must review any failure, queue overrun, coverage loss or output invalidity before assigning GO/PARTIAL/NO-GO.
