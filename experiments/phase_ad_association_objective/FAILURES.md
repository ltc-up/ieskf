# Preserved failures and limits

- Original two-cycle and nearest-valid five-cycle: reused exactly; no backward relabeling.
- Old selected merit: finite boundary jumps; fixed decrease can increase the actual selected merit. This is a mechanism of the discrete objective, not evidence of an algebra bug in the frozen factor.
- New raw solve: 6/13 no-descent/numerical stalls, INCLUDING nominal. Old general KKT helper uses a looser 1e-3 stationarity diagnostic; its boolean does NOT override AD's predeclared 1e-4 interior rule. Raw statuses are preserved.
- The uniform Newton probe allows only <=1e-10 same-energy roundoff nonincrease AND a smaller raw gradient. Nominal measured cost rises 1.82e-11 while analytic gradient collapses. This is NOT presented as measured objective descent; it is a finite-precision stationarity diagnostic. No threshold or trust-radius adjustment.
- Two locally positive-curvature minima: objective consistency does not eliminate nonconvexity or certify the correct pose.
- New mixture removes old duplicate-Target weighting and is density-sensitive; it is a changed sensor potential, not unchanged statistical information. Kernel forces can dominate individual plane-gradient contributions and cancel them.
- New local energy covariance is even tighter; no calibrated probability/recursive filter meaning was verified. Isolated atomic diagnostic discarded.
- No new continuous run, no Z-vz improvement result, no independent position/attitude errors or production qualification.
- Boundary sampling can miss multiple switches inside one cell; detection records are not an exhaustive count. No extra seed or real-data sample was selected to improve a success rate.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
