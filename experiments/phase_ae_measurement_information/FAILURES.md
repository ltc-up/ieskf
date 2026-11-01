# Failures and unverified claims preserved

1. Pure duplicate invariance FAIL: same geometry changes energy, gradients, curvature and nominal pose. It is not linear independent-information multiplication; some synthetic curvatures WEAKEN after duplication.
2. Frozen C/G2 Hessian is nonpositive; no sigma or posterior is manufactured. Full signed matrices/spectra are retained, without an eigenvalue floor.
3. B/axis4_plus and C/axis4_plus fail the original AD solver/precision qualification. Both exit NO_FEASIBLE_DESCENT with large white gradients (~2664/~2287), nonpositive curvature and no active cumulative safety ball. B inner message is positive directional derivative; C inner optimizer says success but the outer same-objective/safety step is not accepted. Neither is relabeled a solution, and the solver is not changed in AE.
4. Other raw stalls/no-descent outcomes rescued only by the SAME previously declared uniform numerical probe are still visible separately. Qualified clusters are finite observed endpoints, not exhaustive mode counts or posterior mass.
5. Fixed-neighbor normal refit changes validity and surface estimate when density changes; it is a separate confound, not evidence that mixture algebra alone explains all real shifts.
6. Large FD stencils straddle nearby Huber breakpoints; real G2 reconstruction error3.32e-5 at1e-5 is preserved, converging~1.94e-9 at1e-6. No original test is rerun or threshold loosened.
7. AD G1/G2 retrospective validation is contaminated. New spatial train-only folds are separate, but low validation radius support and metric-rank conflicts prevent physical accuracy claims. Fixed-witness validation is a disclosed supplementary diagnostic, not a secretly substituted metric.
8. Known-plane X/Y/yaw gain is sampling-generated; statistical posterior qualification FAILS despite10 new numeric tests passing.
9. No continuous Z/vz improvement, independent absolute accuracy, calibrated uncertainty, CUDA/20Hz optimization or production qualification. No covariance scaling/clipping or production edits.


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
