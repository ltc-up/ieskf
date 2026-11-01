# Frozen solver comparison

The adapter changes only the initial frozen-chart coordinate. The original four-loop GN/IRLS direction, fixed-factor Huber line search, dynamic nearest-then-valid correspondence, total guards, per-step trust limits and tolerance remain unchanged. It omits the historical automatic partial posterior creation. Six new interface tests pass; original test suites were not rerun. All five prediction-start historical exit reasons match AH; old accepted endpoints reproduce to below 1e-8. Rejected runs expose their last numerical iterate for diagnostics, but commit neither state nor covariance.

## Original four-loop budget

| Event | Start | Exit | Loops | White GN | Decrement | Pair Jaccard | Strict | Solve ms |
|---|---|---|---:|---:|---:|---:|---|---:|
| E1 | prediction | MAX_ITERATIONS | 4 | 0.44361707 | 30.939004 | 0.084316038 | False | 28.562 |
| E1 | NDT_seed | MAX_ITERATIONS | 4 | 0.0017412311 | 0.10649328 | 0.98758465 | False | 25.354 |
| E2 | prediction | LINE_SEARCH_OR_TRUST_LIMIT | 2 | 4.0987882 | 19.5313 | 0.32918288 | False | 14.987 |
| E2 | NDT_seed | INFEASIBLE_INITIAL_SEED | 0 | 2.0662798 | 6.5645449 | 0.56853726 | False | 0.025 |
| E3 | prediction | MAX_ITERATIONS | 4 | 1.2836196 | 9.3721673 | 0.55082285 | False | 23.372 |
| E3 | NDT_seed | INFEASIBLE_INITIAL_SEED | 0 | 3.9854671 | 7.7105803 | 0.24029727 | False | 0.027 |
| E4 | prediction | MAX_ITERATIONS | 4 | 1.2950374 | 2.3361822 | 0.89314337 | False | 29.244 |
| E4 | NDT_seed | INFEASIBLE_INITIAL_SEED | 0 | 29.727998 | 51.09377 | 0.2137931 | False | 0.024 |
| E5 | prediction | CONVERGED | 4 | 6.2979814e-05 | 0.00010565534 | 1 | False | 10.701 |
| E5 | NDT_seed | MAX_ITERATIONS | 4 | 0.017333237 | 0.030110912 | 1 | False | 11.374 |

Strict qualification requires both prior-white untruncated GN norm and Newton decrement <=1e-4, exact pair identity at the GN probe, support and engineering feasibility. E5's original CONVERGED label checks only GN step and still fails the decrement. A small damped step does not establish stationarity.

## One conditional offline diagnostic

E1/E5 NDT runs showed four fixed-factor accepted decreases without guard hits; the predeclared extension was 16 loops for both starts on these two events. No additional budget/seed/parameter search was performed. Cross-iteration costs are different factor objectives, not a monotonic dynamic energy trace.

| Event | Start | Exit | Loops | White GN | Decrement | Pair Jaccard | Strict | Solve ms |
|---|---|---|---:|---:|---:|---:|---|---:|
| E1 | prediction | MAX_ITERATIONS | 16 | 0.00044297102 | 0.046079361 | 0.99886364 | False | 91.404 |
| E1 | NDT_seed | MAX_ITERATIONS | 16 | 0.00042074743 | 0.046097622 | 0.99886364 | False | 86.456 |
| E5 | prediction | CONVERGED | 4 | 6.2979814e-05 | 0.00010565534 | 1 | False | 11.358 |
| E5 | NDT_seed | CONVERGED | 7 | 3.1986977e-06 | 5.1368955e-06 | 1 | True | 19.252 |

Only E5/NDT passes, at loop 7. E1 remains unqualified at 16: decrement approximately 0.046 and one entering pair at the final GN probe. The two E1 paths approach similar residuals but no claim of identical minima or a proved cycle is made. E5 original start remains at the historical step-only stopping point: a larger loop limit alone cannot repair that stop rule. NDT improves one local example but does not solve the first-frame capture or provide coverage on all five frames.

Timing is a single cached CPU run with one BLAS thread, excluding stored NDT generation, full map loading and live scheduling. The E1 16-loop solve is about 86–91ms; the qualified E5 solve about 19ms. This does not qualify 20Hz end-to-end operation. Raw timing and all failed outcomes are retained.

