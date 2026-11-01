# Phase AI frozen plan

Baseline: 262e22f625aa4b2410e5418429c2ed0bba27a432. Reuse the five predeclared AH full-deskew A events and their stored NDT outputs; no new matching.

A starts at the prediction. B starts at the NDT map-from-base pose, transported to the IMU origin and original right-error chart. Other nine coordinates are the conditional mode of the SAME frozen Gaussian prior at that pose. This is one predeclared completion policy, not another observation. Infeasible completed seeds are rejected without clipping or fallback. Pose-only feasibility is reported separately and does not substitute another seed.

Same original four GN/IRLS iterations, dynamic correspondence, Huber objective, line search, per-step and total guards. No automatic budget extension. Compare fixed prediction-factor scores separately from newly associated scores. Reuse AB tolerance 1e-4 for both untruncated prior-white GN norm and Newton decrement, exact source-target pair witness, support and guards. A small accepted step alone is insufficient.

No posterior is computed by the adapter. Only a strictly qualified endpoint could enter conditional original-model posterior analysis. Neither numerical convergence nor NDT convergence establishes calibrated information. State and P remain uncommitted. No continuous replay. New tests cover the seed interface only; no historical test suites are rerun.

## Conditional diagnostic decision, after four-step results

Only E1/E5 have feasible completed NDT seeds. Both show positive fixed-factor decreases at all four accepted iterations and no guard failure; E1 has final pair Jaccard 0.98758, E5 1.0. These are sufficient to investigate budget limitation, not to certify convergence. One extension is fixed in advance at 16 total iterations for BOTH starting conditions on these two events only. Stop after this test; no further starts, budget search or altered parameters. Each iteration's factor set differs: monotonic cross-iteration dynamic objective is not assumed.
