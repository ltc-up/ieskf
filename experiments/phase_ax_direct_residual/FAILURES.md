# Retained failures and missing qualifications

All full input/state/P/Source/Deskew history/request/result/stdout/stderr files remain private/persistent. Public executed_runs.json preserves actual requested/processed/accepted counts; no failed candidate is overwritten.

- A40v1:38/40 processed,0 updates, repeated max iterations as association objective changes; original2s-loss gate stops execution.
- A40v2:40/40,4 updates then full-bias validity/line-search refusal. The local-map-only noise model over-accumulates conditional information; no validity limit was loosened and no state component was clipped.
- A40v3:38/40,0 updates. Linearized common-pose nuisance can be exploited by nonlinear state rotation and does not solve within limits. Finite-difference gradient is correct for that defined objective; correct derivative is not sufficient for a useful model. Final candidate optimizes the finite SE3 nuisance explicitly.
- Full3D GICP/NDT distribution fails plane observability. Do not feed its Hessian as posterior information.
- B regularized-kernel plane attempt38/40,0updates; exact duplicate CUDA synthetic records change shape/eligibility. Raw-moment reconstruction is a separate model, not threshold relaxation.
- B raw surface lap:531/1468,451updates; no-factor starvation, severe70.5mZ span and large tilt/velocity before loss; three laps NOT_EXECUTED. Frozen alternative-matcher probes are diagnostics, never successful rescue feedback.
- A single/multilap: functional success but right/normal/global engineering-reference regression; not a repair. Persistent map/source correlations remain unmodeled. No new noise search follows.
- First evaluation script had a syntax error, fixed with both failure and successful logs preserved. An early targeted test run skipped3 binaries due missing environment variable; final205 tests run all binaries/no skips.
- Full TIM and empirical-uncertainty paper algorithms inaccessible; only primary metadata/available sections reviewed. Do not claim their exact methods were implemented.
- Automatic approval rejected a batch cleanup of copied AW files. No deletion occurred; all copies retained. This does not affect original data, test results or algorithm qualification.

No independent GNSS clock/installation/map datum certificate, calibrated point/map/scan/cross-frame noise, correct-peak labels, tyre-safety qualification or live ROS realtime test exists. No academic/production superiority claim follows from this branch.
