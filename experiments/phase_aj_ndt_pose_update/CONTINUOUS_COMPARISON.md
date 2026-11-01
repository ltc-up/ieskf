# Continuous A/B/C comparison

A/B are preserved historical same-input runs; C is one new autonomous causal branch. Each has independent nominal state and complete P. Counts are actual submitted updates, not numerical candidates.

| Metric | A: IMU+wheel | B: historical point-plane | C: NDT XY/yaw |
|---|---:|---:|---:|
| LiDAR accepted | 0 | 233 | 66 |
| LiDAR rejected | 284 | 51 | 218 |
| Longest gap without LiDAR update (s) | 14.700 | 1.450 | 5.500 |
| Output Z range (m) | 27.552 | 2.579 | 10.047 |
| Z endpoint relative to initialization (m) | -23.093 | 0.483 | 2.491 |
| Largest actual LiDAR |dZ| (m) | 0.000000 | 0.936400 | 0.310943 |
| Largest actual LiDAR |dvz| (m/s) | 0.000000 | 1.081766 | 0.160987 |
| Final sigma Z (m) | 51.78266 | 0.01673 | 47.59556 |

C retains66/284=23.24%, not most LiDAR events. Its breakdown is129 NDT MAX_ITERATIONS,87 engineering-guard rejects and2 historical-R rejects. Of155 NDT OBJECTIVE_CONVERGED outputs,66 submit,87 fail guards and2 have fitness above the legacy covariance range. Guard hits (overlapping):87 single-step position,58 total position,41 accel-bias,3 single-step rotation. No guard values were altered.

C first accepts at1.10051s. It accepts a contiguous early block, later suffers108 consecutive rejected frames and a5.49978s gap, and recovers on five later isolated occasions (six acceptance-run starts including first acquisition). Last acceptance is12.95056s. Recovery is possible but sustained LiDAR retention fails.

All branches have2,939 IMU events,1,470 accepted wheel updates and1,470 exact100Hz event-time outputs, with no aborted segment. This is an offline deterministic output grid, not proof of wall-clock100Hz scheduling. No nonfinite state or indefinite P was observed.

C's end XY disagreement from B is3.452m (maximum3.989m), heading end disagreement0.05481rad; C vs A end XY disagreement20.796m. These are branch differences, not truth errors. Output increment, speed, innovation and covariance distributions are in key_results.json. B's226 partial MAX_ITERATIONS acceptances remain an unqualified historical behavior, not a correctness benchmark.

Accepted absolute innovation P50/P95/Max: X .02197/.23653/.75002m; Y .04518/.23432/1.38292m; yaw .002452/.007231/.037587rad. Five accepted events use the legacy high-fitness branch f>=3, so66 commits do not necessarily mean66 strong/reliable geometric constraints. NDT can converge to a weak or wrong peak; no independent reference qualifies these poses. No new observation gate is added after seeing these results.
