# Production six-state compatibility — separate audit

The production state is planar position, yaw, yaw bias, forward speed and yaw rate, with independent attitude/height processing. It is not the research inertial 15D state. This phase changes neither production code nor its Q/R/default observations.

The actual `AgedObjectQueue` header was compiled into a private compatibility harness. A queued observation can be popped up to five times. Pop ages the entry before downstream acceptance, so failed attempts consume its lifetime too. Rejecting a new NDT callback does not purge the four remaining opportunities of a previously queued observation. Existing pose covariance is scaled for the repeated-update mechanism. This is not the same as five independent measurements.

Production maintains about 140–150ms of augmented history at 100Hz. The research filter retains about 500ms of timestamped replay. No production API in this audited path reconstructs state and P after retracting an already assimilated observation. Deleting a queue item prevents future attempts but does not undo past state changes. A new callback-only gate cannot provide E4 semantics.

Minimal future changes: unique observation IDs and terminal ledger; explicit pending/retry/expiry policy; no rejected-entry resurrection; timestamped original motion-input journal; restorable state/P checkpoints; deterministic rollback/replay of remaining accepted observations; revision notification for seed and last-trusted-pose consumers. Introduce these behind an opt-in interface and validate repeated-update information accounting before considering a port. The 15D implementation cannot simply be inserted into the planar augmented-state update.

The compatibility harness establishes queue behavior only. No production RMSE, latency, GNSS fix or controller safety improvement is claimed from 15D results.
