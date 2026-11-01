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
