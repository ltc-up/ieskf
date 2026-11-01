# Research status

A complete inertial error-state research prototype exists. Pose/full-state elimination and fixed-association constrained optimization were investigated numerically; those findings are not a deployment qualification.

Dynamic map association remains unqualified for reliable convergence. A first non-converged geometry update can over-contract position uncertainty while leaving velocity uncertainty substantial; propagation then creates strong position–velocity conditional recovery. Large proposed other-state corrections are mathematically explained, but their physical validity is not established.

AA preserved the prediction, wheel and geometry models and investigated actual carried state separately from raw proposals. Authorized de-identified aggregate results are published in the self-contained Phase AA report; full event-level results remain private. Typical geometry improvements do not imply elimination of worst cases; rejecting a proposal is not the same as observing a real state spike.

Next priority: update convergence/association eligibility and credible information/posterior handling, while maintaining a fixed independent preprocessing front end. No independent localization accuracy, long-term statistical calibration, live scheduling or production suitability has been established. The production system remains unchanged.

Research recording policy: each completed or failed phase must include self-contained results/failures, manifest input/config/code identity, index/status updates, reviewed public commit and a normal push to main. Failed authentication or remote conflicts must be reported; never force-push. See AGENTS.md.

## Latest evidence: Phase AB

Final checks distinguish end-of-budget convergence from partial decrease: 26/226 old iteration-limit acceptances pass a conservative local/pair certificate. The first accepted partial update's Z variance contraction is numerically reproduced but not statistically justified; exact Target normalization works, broader correlation remains unresolved. A qualification-only atomic gate passed eight new tests and five frozen implementation checks, yet its one continuous run rejected all 284 LiDAR frames and exactly matched the inertial/wheel-only control. This is a failed operational candidate, not smoother/better localization.

Do not extend replay or loosen gates to rescue acceptance. Next priority is a certified first-frame/local capture solution with consistent association and credible information commitment; preserve prediction/wheel/noise and legal initialization contracts. Fixed-factor solvability alone is insufficient. Independent accuracy, long-term uncertainty calibration, live scheduling and production readiness remain unvalidated.

## Latest evidence: Phase AC

A first-frame failure mechanism is now directly isolated: nearest raw Target selection followed by plane-validity rejection creates an alternating factor deletion despite accurate fixed-factor solves and inactive first-frame safety guards. Sixty outer steps and twelve prescribed starts repeat the same two-cycle. One nearest-admissible selection candidate removes that local hole, but twelve starts (including nominal) converge only to a five-cycle involving remaining radius/plane switches. One other prescribed start gives an interior endpoint; this is not a replacement initializer, not the original-selector solution, and not reliable capture.

No AC continuous replay was admitted. Conditional posterior/state transaction works numerically at the isolated endpoint but remains statistically uncalibrated; Z-vz improvement is unvalidated. Next focus: association/support selection and an optimization/measurement merit model that remain consistent under re-selection, rather than larger budgets, favorable seeds, relaxed safety or covariance/noise scaling. Preserve failures and input/prior identity. Longer replay and production qualification remain unavailable.
