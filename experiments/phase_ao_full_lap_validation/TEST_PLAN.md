# AO pre-execution contract
Baseline: b3de66aa7509b308bf73379892453743fab88676. No Q/R or acceptance threshold search. Production is read only; its existing YAML modification is preserved.

Run order: production timing audit; three-order real timing fixture; input qualification; raw local-motion execution; global qualification; frozen AP diagnostics. This plan was saved before new local/global algorithm execution.

Selection uses input completeness, timestamp validity, recorded sensor continuity and external reference provenance, never new-estimator error. Primary candidate: loop35 raw recording (all three LiDAR, Bosch, scalar wheel, Kistler and both GNSS). Backup candidate: original loop7/loop8 raw recordings; the already accessible loop7-after-940 subset is audited separately. The known-fault loop44 can only support limited execution, not be relabeled normal. R2 is a timing/Source fixture, not an independent local-odom reference.

A candidate closed traverse is the first recorded historical-pose return within 30 m and heading within 30 degrees after 60 s and 3 km accumulated XY travel. These are segmentation rules only, not vehicle accuracy requirements or proof of a surveyed complete lap. If this fails, retain the whole input and report absence of a qualified lap. Never shorten an algorithm failure window. Production poses may select/describe input but cannot be independent truth or ongoing local-state input.

GO requires qualified full-lap input, actual causal frontend/local/global processing, independently supported accuracy/availability requirements and online timing qualification. Missing any requirement prevents full GO. No independent reference => no accuracy RMSE. Unknown GNSS datum/fix/lever/clock/noise => D/E NOT_QUALIFIED. Unknown public NDT convergence/seed/source contracts => no trusted anchor. Distinguish received, accepted shadow, and trusted publication.

Tests: real Header/recorded arrival/historical interpolation comparisons; entire raw local stream with existing 15D propagation and wheel model, original TF only; invalid timestamps rejected; full P retained. Abort/retain failure at existing IMU-gap gate, no state resets. Global modules receive copied poses only. AP freezes all AO code/configuration; faults never tune the baseline.
