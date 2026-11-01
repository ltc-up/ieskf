# CPU offline performance

|Scope|P50|P95|P99|Max|
|---|---:|---:|---:|---:|
|One complete dynamic cost+gradient evaluation, ms|8.572|9.775|10.921|78.364|
|Raw full13-start solver per seed, ms|4722.922|10951.102|16610.083|18024.828|

Nominal raw solve=18024.828ms; uniform nominal endpoint curvature/precision diagnostic adds1632.453ms. Median diagnostic cost=1271.305ms (includes the three Hessian probes, NOT a production update). Map tree construction/loading excluded; the first solve includes on-demand normal warming, later starts share that cache. These 13 runs on one frozen frame are not an independent production latency population. Some offline boundary-analysis work overlapped the first solve; no dedicated uncontended latency benchmark is claimed.

The 50ms/20Hz total-update budget is **NOT met**. A ~9ms individual merit evaluation does not imply a 20Hz nonlinear update: full solves require many evaluations. No ROS scheduling, CPU/GPU transfer, GPU query, live Deskew or continuous state propagation is timed. Avoid declaring GPU/real-time potential from the cost of one vectorized query. Mathematical convergence progress is not an engineering real-time pass.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
