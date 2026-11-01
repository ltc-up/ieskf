# Experiment index

Public entries are separate redacted summaries. Immutable full records and checksums are local under ignored data/history. Synthetic numerical qualifications do not establish sensor accuracy.

| Phase | Question | Main conclusion | Qualification | Public record |
|---|---|---|---|---|
| U | Full pose geometric factor | Direction and cross terms represented; repeat-point information can be overcounted | Numerical prototype | [U](../experiments/phase_u/README.md) |
| V | Full inertial state and iterative geometry | Complete offline prototype; partial-update acceptance needs scrutiny | Conditional replay only | [V](../experiments/phase_v/README.md) |
| W | Front-end contract and convergence | Independent preprocessing preserved; guards and association change matter | Conditional provenance | [W](../experiments/phase_w/README.md) |
| X | Joint MAP and line search | Fixed-association objective consistent; small feasible steps were missed; fix did not establish convergence | Local numeric only | [X](../experiments/phase_x/README.md) |
| Y | Constrained MAP and elimination | Full/block methods agree locally; dynamic outer loop and constrained posterior remain unqualified | Local optimization only | [Y](../experiments/phase_y/README.md) |
| Z | Cross covariance and trusted update | Large other-state proposals follow prior correlations; numeric posterior is not statistical calibration | Conditional numeric | [Z](../experiments/phase_z/README.md) |
| AA | Actual versus proposed vertical motion; migration | Portable code reproduces legacy decisions; proposed/committed motion and covariance evolution must be reported separately | Diagnostic only | [AA](../experiments/phase_aa_z_validation/FINAL_CONCLUSION.md) |
| AB | Convergence qualification and first Z-information contraction | Numerical postchecks expose partial updates; qualification-only continuous run loses every LiDAR constraint | Numeric PASS; constraint retention FAIL | [AB](../experiments/phase_ab_geometry_qualification/FINAL_CONCLUSION.md) |

## Phase AA completed execution record

Goal: separate candidate vertical-speed requests from committed state changes after the identified improved preprocessing chain. Executed: corrected and legacy continuous diagnostics, common-frozen-prior Source comparison, migration smoke checks and exact historical state/P parity verification. Main finding: large proposals can be rejected while actual committed changes remain smaller; early iteration-limit covariance tightening and subsequent conditional amplification persist. Final qualification: numeric parity PASS; old/new causal equivalence CONDITIONAL; convergence, accuracy, statistical consistency and production suitability NOT_VALIDATED. Reports: [self-contained conclusion](../experiments/phase_aa_z_validation/FINAL_CONCLUSION.md), [failures](../experiments/phase_aa_z_validation/FAILURES.md), [manifest](../experiments/phase_aa_z_validation/manifest.json).

## Phase AB — completed, negative continuous outcome

Goal: audit all iteration-limit geometric updates, first-frame covariance tightening and an eligibility-only remedy. Executed: reconstruction of 226 partial plus 7 solver-converged acceptances, one rejected-proposal diagnostic, four fixed-factor refinements with 48 pose probes, eight new tests, five frozen implementation checks and one controlled continuous candidate. Findings: 26/226 have final local/strict-pair evidence; first posterior shrinks Z variance about 4546x despite nonstationarity; candidate keeps 32/233 frozen acceptances but 0/284 in continuous recursion and equals the no-geometry control. Qualification: numeric reconstruction/atomic semantics PASS; continuous constraint retention and longer-window eligibility FAIL; accuracy/statistical consistency NOT_VALIDATED. [Report](../experiments/phase_ab_geometry_qualification/FINAL_CONCLUSION.md) · [manifest](../experiments/phase_ab_geometry_qualification/manifest.json) · [failures](../experiments/phase_ab_geometry_qualification/FAILURES.md).
