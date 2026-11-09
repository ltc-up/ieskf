# AU3 — Controlled counterfactuals and their qualification

Execution follows AU1 equivalence and AU2 real-history decomposition. Representatives are the predeclared median frames: control1→1144, right3→1361, right9→883. No GNSS error or tuning selects them. All factual algorithms and parameters remain frozen.

## A. Exact original matrices

The original full15D update, H, S, K, delta, Joseph and right-error reset were independently reconstructed across all1468 frames, not just the chosen three. Maximum matrix/delta discrepancies below7e−16; finite-difference reset checks below8e−12. This verifies numerical implementation under the given model and does not certify real uncertainty.

## B. Fixed-state PSD sensitivities

Five predeclared matrix cases use original P/R, P×.5, P×2, R×.5, R×2. Congruent positive global scaling preserves SPD and all correlations; no rows or cross terms are deleted. These matrices are analyzed **outside the factual replay**, never installed as new settings. No GNSS score is computed and no scale is selected.

| Representative | Original lateral unit gain | P×2 or R×.5 | P×.5 or R×2 |
|---|---:|---:|---:|
|control1|.1003|.1819|.0528|
|right3|.0942|.1721|.0495|
|right9|.1078|.1944|.0570|

The same gain can result from larger P or smaller R, despite different posterior covariance magnitudes. Gain alone cannot identify which covariance is physically miscalibrated. Increased correction is not proven increased accuracy; engineering fitness R remains uncalibrated.

## C. One-information-event interventions,250ms

Each baseline starts at its exact saved full-state/P/held-IMU checkpoint and processes the same canonical Header events for250ms. All three baseline means and P match the original history **exactly** at every processed event (86/84/86 events). Cases skip one entire selected NDT update or the first subsequent wheel update. The replaced event is a clock-only advance, preserving the integration split and original held-IMU age. Skipping the event altogether would also alter quadrature, an unwanted second intervention.

All future NDT final poses and engineering R are fixed recorded observations. The branch recomputes its own prediction and candidate seed, but does **not** rerun Deskew/CUDA. Seed changes are logged; actual future matcher output is UNKNOWN. A changed covariance naturally changes later gains; no selective gain removal, covariance cross-term deletion, bias clipping or parameter replacement is used.

Signed lateral branch-minus-baseline differences use the fixed original representative body axis:

| NDT skipped once | Immediate difference m | Next native prior difference m | Next/immediate | Difference at≤250ms m |
|---|---:|---:|---:|---:|
|control1|−.001311|−.001573|1.200|+.001108|
|right3|+.003267|+.003484|1.066|+.002923|
|right9|+.012876|+.013424|1.043|+.005747|

At these examples, the previous NDT correction's influence **persists** to the next prior; it is not erased by propagation. Subsequent observations and state coupling alter that influence, including a sign change in the control example by the ≤250ms window end. This is different from a factual relative prediction/observation discrepancy recurring because both vehicle and NDT estimate move.

Skipping the one subsequent wheel event changes the next-prior fixed-axis position by approximately+.250mm/−.017mm/+.430mm for1/3/9. Maximum future seed-position changes are1.10/.184/2.32mm; the NDT-skip cases reach26.5/9.90/24.0mm. Thus a direct single wheel correction is small in these examples, but indirect propagation and the aggregate history of thousands of updates are not ruled out. Full branch mean/P, Bias/velocity changes, acceptance and own-seed differences are retained privately; public JSON contains aggregates. No unexpected future NDT rejection occurred.

**Qualification:** A VERIFIED numerics; B VERIFIED matrix sensitivity under uncalibrated assumptions; C CONDITIONALLY interpretable fixed-observation effect. No new true CUDA closed loop, matched-peak accuracy, independent physical truth or full-lap improvement is claimed. These experiments provide no sufficient basis for changing production Q/R or the physical motion model.

The .25s is the declared cutoff, not a forced new integration endpoint. counterfactual_results.json records each actual final sensor-event span and event count; no requested duration is substituted for processed coverage.
