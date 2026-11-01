# Deterministic frozen events

Selected from the existing AA window before ablation outcomes: first; maximum absolute raw candidate dvZ among rejected frames; largest old accepted dvZ; minimum absolute yaw rate among accepted old CONVERGED events; maximum speed×absolute-yaw rate; maximum in-scan RP change. Deduplicating overlapping roles gives5 events. All original hashes/physical time/message identities are preserved privately; aliases below remove absolute time/track identifiers.

|Alias|Roles|Speed m/s|abs yaw rate rad/s|scan RP change rad|
|---|---|---:|---:|---:|
|E1|first, max_speed_yaw|23.110|0.409803|0.001827|
|E2|max_rejected_raw_vz|23.444|0.376293|0.000630|
|E3|max_old_accepted_vz|23.882|0.354302|0.000329|
|E4|max_scan_RP|41.130|0.001047|0.009373|
|E5|least_curvature_converged|62.750|0.003491|0.001917|

E1 covers both first update and greatest speed×yaw product in this limited window; it is not a claim to cover every high-speed turn. E5 is the least-curvature old-converged sample, not a ground-truthed straight/zero-pitch segment. E4 has largest endpoint RP change; no success-based seed/source selection occurred. E1/E2/E3 are close neighboring events; E4/E5 are separated later events, so differences between their absolute heights must not be called adjacent-frame jumps.

For EVERY A/B/C evaluation, each alias uses exactly the SAME AA original per-event prior/P and historical IMU/wheel prediction. Later B/C events do NOT inherit a hypothetical earlier B/C posterior. This isolates immediate source sensitivity, not autonomous closed-loop feedback. No cohort extension or all-bag search was performed.
