# Failure and uncertainty record

- Eligibility-only continuous candidate failed operationally: all 284 geometry updates rejected; carried state/P exactly equal to the inertial/wheel-only control. The experiment stopped after this single candidate window.
- 25 tentative geometric updates failed final local/association qualification; 259 original support failures occurred before the postcheck. No relaxation, velocity clipping, noise edit or extra iterations were introduced.
- First fixed-factor local refinement does not remain stationary after reassociation. Two representative unconstrained reference solutions violate the existing gyro-Bias guard and are diagnostic minima only.
- Exact Target repetition is normalized correctly; correlation and map/normal/shared preprocessing uncertainty remain unresolved. Block sensitivity is not a calibrated covariance model and does not prove a single-plane interpretation.
- Exact pair equality is a conservative witness, not a necessary physical criterion or global stability proof. The local tolerance is numerical and inherited, not a calibrated hypothesis test.
- No independent positioning accuracy, cloud-height improvement, long-term consistency or live scheduling qualification was performed. No claim that all 200 uncertified partial acceptances are physically incorrect.
- No unhandled numerical execution error occurred in the completed diagnostics/new tests. Failed scientific/engineering hypotheses remain failures even with finite SPD covariance.
- Delivery history: previous phases encountered HTTPS authentication failures; the final response for this phase must report the actual current push outcome, never infer availability from a local commit.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
