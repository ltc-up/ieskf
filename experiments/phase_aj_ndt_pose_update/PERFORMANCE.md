# Performance and timing scope

One C continuous replay consumed14.700655s of sensor time in2.512421s wall time. The GPU map is cached once; active snapshots update without rebuilding the full Target. Counts:284 alignments,6,163 propagation subintervals,1,470 wheel updates. A/B timings are historical and not a fresh same-load benchmark.

| Component (ms) | P50 | P95 | P99 | Max |
|---|---:|---:|---:|---:|
| NDT align synchronously measured | 2.6896 | 3.6464 | 4.2132 | 5.6815 |
| NDT RPC including extra quality | 3.1302 | 4.1750 | 5.7385 | 276.9480 |
| LiDAR callback, excluding prior Source load | 3.7595 | 5.1531 | 19.8106 | 277.7753 |
| ESKF saved-prior posthoc audit | 0.0626 | 0.1023 | 0.1170 | 0.2191 |

Source file load is separately measured: P50 .1854ms, P95 1.4509ms, P99 2.0163ms, Max2.7864ms. Do not add marginal quantiles and call them a combined quantile. Source binary preparation is included in callback time (P95 .2884ms); map build itself is5.0873ms. The cold first callback is277.775ms, including process/context/RPC startup. Two additional callbacks take85.517 and119.539ms. Their matching kernels are only1.83 and3.07ms; unprofiled CPU/IPC/scheduling delays must not be attributed to the optimizer without evidence.

All3 >50ms callback events are retained. P95/P99 show useful offline computational potential, but the measured callbacks themselves violate a uniform20Hz budget; full pipeline adds Source handling and excludes live Deskew, bag ingestion, ROS scheduling and real asynchronous delays. Therefore end-to-end20Hz qualification is NOT_VALIDATED, not PASS. No scheduling/backlog/drop-free claim is made from sorted offline replay. All284 events were processed offline and all100Hz output slots were generated.

Inertial prediction P95 .09670ms; wheel update P95 .06526ms. The inner ESKF timer was overwritten by the enclosing callback timer during the original run, so its isolated cost was not retained. The table's ESKF row is a separately labeled153-case saved-prior audit, not a fabricated in-run measurement or another continuous replay. Those audit decisions/P reproduce the actual run.

Peak main-process RSS reported338.3MiB includes loading historical reference data; it excludes external service/GPU memory. GPU peak memory was not instrumented. The inherited generic Diagnostics field initially said GPU not used; this metadata is inapplicable to the external service, and the public summary explicitly corrects the label. Raw original evidence is preserved.
