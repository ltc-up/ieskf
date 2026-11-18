# B timing and resource results

Synthetic six-frame test executed under the common GPU flock, sequentially running B-P/B0/B-M/B-K. Five actual matches per LiDAR variant all passed the frozen engineering validator. This verifies CUDA wiring and local target replacement; it is not a real scan workload or isolated scheduling certification.

|Variant|Full callback step P50 ms|Target build P50 ms|Align P50 ms|Bridge P50 ms|
|---|---:|---:|---:|---:|
|B-P|.075|—|—|—|
|B0|22.828|1.138|2.141|4.044|
|B-M|63.528|1.123|2.033|4.109|
|B-K|52.156|1.198|1.788|3.614|

B0 Target ~15525–15585 points, 929–948 valid Gaussian voxels; B-M ~15535–16753 and 933–1072; B-K retains the initial keyframe in this high-overlap synthetic scene. Native extra quality evaluation ~.055 ms median; destruction .103–.131 ms. The substantial CPU crop/voxel/history cost remains visible. Multiframe synthetic P50 already exceeds 50 ms. No normalization or hidden removal of target rebuild costs is allowed.

Real P50/P95/P99/max, isolated single-variant scheduling, CPU/GPU/RSS/VRAM, complete A0+B period, actual 100 Hz state output age/continuity are pending. Current callback is synchronous for input lifetime correctness. Production 100 Hz output must read the committed common-runner snapshot without waiting for this GPU work. History is bounded to three scans/one second; no large cloud archives. Dense point masks retain hashes and bounded small-ID samples in logs, with full actual point-ID arrays retained only in live bounded history.

Additional B-K-MR synthetic rerun: five coarse3→fine1.5 registrations all accepted. Combined full-step P50 53.755 ms, combined target build 1.612 ms, align 4.006 ms and bridge 6.632 ms. No claim of speedup; native two-level costs are summed. Shared input copy/hash preparation was initially outside per-chain step timer; bridge now explicitly reports input_prepare_ms, all_variants_ms from before preparation and, for a single variant, full_callback_variant_ms including preparation. Earlier synthetic per-chain numbers remain labeled step times rather than claiming complete A0+B period.

Independent 332-frame early-prefix audit of the initial dense tuple/set keyframe implementation found B0 step P50 12.61 ms, B-M 34.83 ms, B-K 110.88 ms and B-K-MR 112.38 ms. These are a running-prefix observation, not full-run or isolated end-to-end qualification. A pure mathematical equivalent occupancy-cache/vectorization optimization was authorized before completing continuous evaluation; its separate reference-policy tests pass. The initial slow implementation remains preserved as a failed performance prefix; final full-run metrics must identify the actual frozen code version.

## Actual-clock KINEMATIC_BK first attempt: failed on both maps

The separately executed real raw-input/GPU/ROS observer failed after eight 0105 local commits and nine 1013 commits. 0105 stopped at SEVERE_INPUT_DELIVERY_DELAY (100.018 ms); 1013 stopped at EXPIRED_OR_INVALID_DESKEW_HISTORY. No held/rollback/delivery limits were relaxed. Original failure outputs and source identity remain available.

|Map|Completed local scans|Dispatch→commit P50/max ms|Published/received packets|Valid Pose packets|Health-only packets|
|---|---:|---:|---:|---:|---:|
|0105|8|69.03 / 159.83|649 / 649|51|598|
|1013|9|61.07 / 171.77|657 / 657|56|601|

Those original cycle timers started after CPU prior/history preparation and are therefore incomplete 50 ms periods. Cold first local matching was approximately 127/139 ms, and subsequent worker costs were commonly 40–60 ms with an additional main-thread commit delay. Complete DDS receipt does not imply usable fresh Pose coverage; most packets contained uninitialized or invalid-input health rather than a Pose. Real-time qualification is NO-GO for this original harness.

A separately named v2 preserves the cold start and all limits, caches immutable already-validated motion samples, stops pure Legacy computation only after its one-time initialization role is formally committed, and measures the complete preparation→commit period. All 7,798 actual motion query records and frozen deltas are exactly equal after 118,473 CDR-identity checks. Nineteen CPU tests pass. The measured component speedup is not a complete-cycle result; two full actual-time retries are queued. See [v2 validation](KINEMATIC_REALTIME_V2_VALIDATION.md).
