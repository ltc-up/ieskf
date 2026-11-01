# AB5 — one admitted continuous counterfactual: FAILED coverage

Admission was recorded after objective/P reconstruction passed, eight new tests passed, five frozen implementation checks passed, and the first-frame nonstationary-posterior mechanism was identified. Expected frozen coverage loss was disclosed before running. Exactly ONE candidate replay was then executed; AA and the no-geometry control were reused without rerun. The original nominal 14.70s event stream, initialization, all sensor/geometry parameters and original guards were unchanged.

| Metric | AA saved baseline | AB eligibility-only candidate |
|---|---:|---:|
| LiDAR committed / rejected | 233 / 51 | 0 / 284 |
| Committed local convergence | 7 old step-criterion labels | 0 |
| Committed strict association certificate | Not imposed by AA | 0 |
| Largest committed abs(delta Z) | 0.936400 m | 0 m |
| Largest committed abs(delta vz) | 1.081766 m/s | 0 m/s |
| Maximum carried abs(vz) | 2.213607 m/s | 3.480163 m/s |
| First committed/retained sigma Z | 0.015979 m | 1.077427 m |
| End sigma Z | 0.016728 m | 51.782665 m |
| End Z change from initial state, NOT truth error | 0.483381 m | -23.093366 m |
| Max output abs(rho(Z,vz)) | 0.998086 | 0.961230 |
| Diagnostic update P50 / P95 / P99 | 10.923 / 17.904 / 19.807 ms | 10.832 / 68.023 / 75.946 ms |
| Output snapshots | 1,470 | 1,470 |

The candidate rejects 25 historical-solver tentative acceptances for both insufficient final local convergence and absent strict association witness. The other 259 terminate with original LOW_SUPPORT before qualification. They were NOT all tested and found locally nonconverged; their post-solve conditions were never reached. First low support occurs about 0.200401s after the first LiDAR event. Since all 25 tested tentative candidates also fail local optimality, exact-pair conservatism alone does not explain this collapse.

The entire carried position, velocity, Bias and covariance history matches the saved IMU+wheel-only control EXACTLY, maximum error zero. The filter receives no LiDAR correction throughout the window. Removing the first unqualified contraction therefore removes the baseline's bootstrap path, and the unchanged four-step solver cannot reacquire a qualified local solution before support is lost. This is an observed prediction/geometry interaction, not proof that the initial physical pose or IMU model is wrong.

All candidate outputs are finite and P stays numerically positive, but that is not success: constraints vanish, uncertainty grows and the maximum carried vertical speed is larger. Zero committed velocity increments merely reflect zero LiDAR updates. The relative Z change above is not an independently measured error. Output spacing is an offline replay property, not an online 100Hz scheduling validation; timings include additional diagnostic identity queries and are not production performance.

Result: `CONTINUOUS_CONSTRAINT_RETENTION=FAIL`. No thresholds were loosened, no second window or extra run was attempted, and a longer replay is NOT eligible. Private full-state/covariance curves remain excluded from GitHub; this aggregate table provides the complete result needed to assess the failure.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
