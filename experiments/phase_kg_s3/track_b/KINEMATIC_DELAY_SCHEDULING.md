# KINEMATIC_BK causal scheduling and review interface

This addition implements the approved kinematic contract in new files. Original Legacy B-Prediction, B0/B-M/B-K/B-K-MR modules, labels and records remain intact. Old B-K means keyframe Target. New variant identities use `predictor=KINEMATIC_BK` and an independent Target axis.

## Measurement release and interval query

`kinematic_predictor.py::IntervalKinematicPredictor.query(prev_ref,ref,query_available,original_cloud_available)` returns VALID with a from-I body-twist SE(3) increment, DELAY with no partial seed, or an audited IRRECOVERABLE_INPUT_GAP. A sample must have both header no later than its ZOH segment start and original recorder availability no later than the actual query. No interpolation or future value fills a gap. Fixed zero bias is immutable. Bosch angular velocity is already rotated once by the common input adapter; the predictor does not rotate it again. Wheel x is recorded base velocity, with no repeated lever correction.

The approved scheduling clarification supersedes treating the first missing admissible sample as terminal: DELAY retries on the next actual raw input release. There is no new timeout. Original cloud availability remains explicit; later retries retain their later availability and full delay. The Source/dense packet is copied at original release, with original hashes. Later scans queue behind the head and cannot advance its history. When committed, the same frozen Target builder uses only processed history and the newly legal causal prediction. Targets are generated at commit rather than fabricated while waiting. Physical gap, invalid input and resource failure are separate states.

The original IMU 25 ms and Wheel 100 ms hold limits remain. Only the complete original bag header/availability/identity audit can establish an unrecoverable physical header gap. Future metadata may classify coverage, but provides no values to an earlier query. EOF alone leaves DELAY_UNRESOLVED_AT_CALLER_END. If needed, the runner reads actual later motion CDRs at their true recorder availability, writes a separate tail ledger, and admits no new LiDAR/A0 state after the requested end. Main input identity remains unchanged. Audit metadata also records raw frame sets and Wheel variance validity.

## State and bounds

Each interval starts from identity. Committed interval increments concatenate to Ukin. The first legal Legacy B0 O0 supplies a single fixed H_init so the initial O/ROI axes agree. No later Legacy/global posterior pose, velocity or bias enters KIN. T_map_from_O is explicit evaluation metadata. Each Target variant owns a separate matcher/history/O chain; proposals are subject to the frozen engineering validator and rejected proposals do not become accepted anchors.

Motion records are capped separately at 1024; immutable pending scans at 10 and 128 MiB. Pre-initialization arrivals are causally pruned to the newest held header at/before initialization per stream, retaining any already-arrived future headers. RESOURCE_FAILURE never masquerades as GAP. Disk monitoring covers the entire B archive root with a total cap of 2 GiB, not each run. No per-frame Target arrays or full selected-point ID lists are persisted.

## Runner and independent review

`kinematic_causal_runner.py` accepts the original A0 config, `b_modes=["B0"]`, `shared_module_path`, and `b_branch_resource_root`. `kinematic_targets` selects NONE/PREVIOUS/ROLLING/KEYFRAME/KEYFRAME_MR; each is a complete independent chain. `frames.jsonl` retains A0/Legacy B0 schema. `kinematic_events.jsonl` records DELAY, COMMITTED and terminal events under original ref identity. COMMITTED contains original/query availability, interval provenance, Source/dense hashes, explicit first gauge, prediction, complete proposal and final pose per variant. Offline evaluation joins by ref, never by retry arrival frame.

`kinematic_paired_probe.py::FixedHistoryPairProbe` is an independent diagnostic wrapper. Optional `paired_cases` must contain exact preregistered `ref_ns` and unique `case_id` (with source/lag provenance). It reuses Legacy's actual Source/Target/request/result. The alternate seed is the same Legacy previous O times the KIN increment for the same interval. A second Target column rebuilds the same historical anchors with only the KIN ROI center changed. Four seed/ROI cells include the original result plus three extra alignments. Arrays remain in memory; only hashes/recipes/proposals are logged. These results never update any state. DELAY at original release yields no fabricated future seed. Missing cases and empty targets remain explicit. BB 100 ms historical comparisons require a separately labeled historical interval; B0 itself remains strictly the immediately preceding scan.

This runner measures actual computation but does not advance recorder clock by computation latency. It cannot establish a 50 ms complete cycle or real 100 Hz publication. A later pose-only wall-clock harness must evaluate B outputs without inventing a 15D velocity/bias/P.

## Current validation scope

59 CPU tests cover exact SE(3), 3D attitude, physical-z motion, ZOH/time boundaries, future-value exclusion, delay/gap distinction, first-axis parity, immutable Source retention, strict queue ordering, prehistory pruning, separate resource caps, and fixed-history 2x2 isolation. The complete real motion metadata audit found the expected raw frame sets and no initialization-to-requested-end physical gaps. GPU integration and complete continuous KIN trajectories are pending; these tests are not accuracy or timing qualification.
