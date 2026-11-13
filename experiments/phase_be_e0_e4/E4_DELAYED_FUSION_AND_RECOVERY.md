# E4 — Observation lifecycle and bounded historical revision

`ManagedReplay` adds PENDING, ACCEPTED, REJECTED and EXPIRED records keyed by unique observation ID and timestamp. Admission is final only after the actual update executes. Duplicate IDs and correlation groups cannot add information twice. Rejected observations are excluded from retained replay, including after a later revision invalidates another domain-limited update. Terminal tombstones survive checkpoint eviction.

For an already accepted observation inside the retained window, retraction removes its event and replays the remaining original sensor/pose events from the unchanged oldest checkpoint, including full covariance. It never subtracts an old state increment, resets the filter to a matched pose, or clears all observations. Unit tests compare mean and full P against a run that never admitted the revoked observation. The historical observation timestamp is not replaced by callback time.

Beyond 500ms there is no retained checkpoint from which this implementation can undo a committed update. The runner preflights all affected branches and leaves all states and weights unchanged when any required history is unavailable, recording REVOCATION_OUTSIDE_HISTORY. This is an explicit limitation, not a successful rejection. Expiry of a new delayed observation is distinct from retraction of an already fused one.

The robust replay injects a 30-frame no-admission interval at indices 300–329 and requests historical retraction every 200 frames. These are controlled lifecycle faults, not an error-NDT classifier. An accepted wrong result is not identified automatically by these injections. Neither this mechanism nor geometric conditioning establishes which real scan is wrong.

The frame contract uses `T_MB=T_MO T_OB`, with `T_MO=T_MB inverse(T_OB)`. A separate motion-only odom chain stays continuous during map correction and mode selection; map/path transforms carry the same monotonically increasing version and reject mixed versions. This is not low-pass smoothing. Odom drift is unbounded in the long term and map-mode switches may still change global pose. No controller consumes this experiment's interface. Cross-covariance between map correction and odom, a controller-in-the-loop test and path/command safety bounds remain unvalidated. High-speed automatic global relocation remains NO-GO.

The full replay and precision tests must be read separately: numerical rollback correctness in the retained event model does not certify localization accuracy, real-time scheduling or control safety.

## Executed fault-replay result

The1468-frame robust run completes. Seven historical-retraction groups are requested; five score revisions and five state revisions per branch succeed. One request is explicitly beyond retained history; remaining requests without an accepted target are not successful retractions. Final ledger: NDT1242accepted/36rejected/190expired, GICP1184accepted/111rejected/173expired. The59 injected branch refusals reported during processing differ from post-revision terminal counts.

After the30-frame no-new-LiDAR interval, native trajectory divergence from the matched closed-loop control reaches0.496m. First accepted selected-mode update is index330, with base-position correction0.360m. This is closed-loop counterfactual divergence, not certified pure IMU drift; IMU/wheel remain active and matcher/latency feedback differs. Neither value is a safe high-speed jump threshold. Rejecting a0.36m correction merely because a Go2 example used0.3m would discard this recovery event.

Full-cycleP95 is65.90ms, with299/1468 cycles above50ms. The robust three-lap attempt is incomplete at4035/4134 and is a failed qualification. Map/odom/path algebra is tested, but no vehicle control loop, global mode-switch safety bound or joint map/odom covariance has been validated.
