# Frozen identity and causal branch contract

Baseline: ac797fbf6905fdeba57a0660f23e8ca9ca6796da. The native AA window contains 284 unique full-deskew Sources, 2,939 propagated raw IMU events, 1,470 accepted wheel observations and 1,470 output ticks. Actual endpoint duration is 14.700655404s; output ticks cover 14.70s at exact10ms intervals. The existing A inertial/wheel control and B AA partial geometry replay are reused; neither is rerun. No new bag extraction, Deskew, point-plane MAP or map model was run.

All A/B/C initial nominal fields and complete P agree exactly, and all 284 event times/Source hashes agree. Before C's first accepted LiDAR update, its saved states/P match A exactly. Input/config/Source/backend digests are in manifest.json; complete raw payloads remain private.

NDT receives each original native Source through the unchanged float32 backend interface. The stored geometry baseline uses saved float64 native points; casting preserves point IDs/order/physical frame but does not assert scalar-bit identity to the geometry representation. This is the same disclosed AH interface conversion, not a different Deskew Source.

The static Target bytes and DIRECT7/GLL/resolution/iteration/fitness configuration are unchanged. One GPU map is cached for C. The original cylinder radius/vertical offsets and translation-triggered map-region policy are retained, but its center and updates use ONLY C's current/past prediction. Reusing a historical branch's crop center would leak another branch; numeric crop centers therefore need not equal B. C height drift can also affect active-region queries under this unchanged policy; this phase does not independently isolate that feedback.

For every one of the284 matches, the logged float32 seed equals the current C map-from-base predicted transform cast to float32 exactly. Reconstruction from serialized state quaternions introduces at most2.85e-14 double roundtrip difference. No A/B future pose or NDT trajectory is supplied. Original historical fused pose is used only for the common frozen initialization, with its correlation disclosed. NDT is a same-LiDAR pose observation, never independent truth.

One private persistent request/response wrapper invokes the existing unchanged CUDA library. It is not a new matcher/backend; local adapter and restricted headers/code stay ignored. One same-input, same-seed adapter parity fit reproduced AH final transform and fitness exactly before the C replay. Actual new NDT alignments:285 (one parity check plus284 causal matches). No historical covariance/optimization suite was repeated.

Offline physical/Header-time sorting and ZOH propagation remain unchanged. No online message-latency rollback or ROS/TF publishing is involved. Source, propagation state and measurement share the native max-scan reference. Full raw timestamps, lever, matrices, trajectories and SDK payloads are excluded publicly.
