# Frozen candidate A/B

A is the saved AA state/P; B applies ONLY the new eligibility decision to the SAME saved predictions, Sources, map and original candidates. No prediction-history changes are introduced in this table.

| Group | AA accepted | B retained | B rejects old acceptance |
|---|---:|---:|---:|
| Iteration-limit label | 226 | 26 | 200 |
| Old solver-converged label | 7 | 6 | 1 |
| Total old acceptances | 233 | 32 | 201 |

Of the 226, 26 already meet final local and strict-pair numerical criteria; 22 have identical last/final pairs but insufficient local stationarity; 178 have changed pairs and lack joint qualification. These categories identify evidence, not complete causal labels. It is not justified to call all 178 physically unstable or all 22 pure budget failures. In this dataset the exact-pair witness rejects NO additional event among those passing both local criteria.

Five fresh frozen implementation calls cover first update, rejected large proposal, largest committed vertical-speed correction, borderline old step-only convergence and a well-converged end-of-budget event. All match the reconstructed decisions. First and large-correction updates reject; the small final-residual sample commits its exact original state/P; the already rejected large proposal remains rejected. No constraints are relaxed.

This is a successful semantic/numerical test, not a filter-quality improvement. It predicts severe coverage loss: only 32/233 old acceptances survive, and this does NOT predict the count after continuous predictions change. The continuous result is reported separately and is a failure of operational constraint retention.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
