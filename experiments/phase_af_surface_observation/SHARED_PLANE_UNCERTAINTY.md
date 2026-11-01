# Shared plane uncertainty

Executed: low-rank covariance solves versus dense SPD R; full non-diagonal Sigma; finite-difference Jplane; anchor transport; explicit18D state+plane latent solve versus15D marginal posterior; dense Kalman versus Joseph; known-law Monte Carlo5000 draws (fixed seed20261101). Empirical covariance relative Frobenius error=0.04602396 against predefined0.06 numerical check. This is synthetic validation, not fitting real map noise.

The latent parameters represent coherent plane tilt and offset affecting every assigned Source residual. Source independent noise can average down; shared offset cannot. For the symmetric single horizontal plane with sigmaZ_prior=1m,

`Var(Z|r)=1 / (1 + 1/(0.1² + 0.2²/N))`.

Increasing independent Source samples from9 to1089 approaches variance0.00990099m², not zero. Exact aliases are one measurement, not N new independent errors.

Different independent physical planes can add information. Splitting ONE plane into1/2/4/8 patches must NOT create independent eta variables. With81 samples, correct shared-parent sigmaZ stays0.10190609m; the intentionally wrong independent-patch version shrinks to0.101906/0.073918/0.054636/0.041730m. This counterexample passes as a diagnostic of model misuse; it is not a recommended method.

Real extraction uses one TLS plane per visited grid cell, exact unique map support, then neighboring coplanarity grouping and whole-component TLS/member validation. Thresholds come from unchanged research normal/Huber settings, not a performance search. Grouping is a deterministic geometric hypothesis, not a recovered physical plane identity. Baseline5m has96 accepted groups,4 unresolved components excluded,26 groups touched by prediction. The largest Source group carries55.26% of residuals but only9.27% of Z information under shared noise: repeated support is not counted as independent map planes.

Uncertainty provenance: support/eigenscatter are measured fit diagnostics; they are NOT a map covariance estimator. Across accepted baseline groups RMS fit scatter min/median/P90/max=0.03485/0.26726/0.44351/0.54784m. This can exceed the transferred offset std0.1m, and demonstrates why the synthetic Sigma is not credible calibration. True map measurement noise, surface curvature, segmentation error, common datum error and cross-group correlation are missing. A repeated scan/reconstruction or independent plane reference is required. Do not bootstrap individual Target records as independent plane truth.

Qualification: SHARED_ERROR_NUMERIC=PASS; REAL_PLANE_IDENTITY_AND_COVARIANCE=NOT_VALIDATED.
