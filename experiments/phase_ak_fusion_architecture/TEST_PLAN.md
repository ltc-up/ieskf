# Phase AK test plan — 2026-11-08

Scope: architecture research; no production changes, private replay, map export, solver retuning or deployment. Baseline: 97df23be8b110164e29a48183bde70266ace4133. Existing local main was clean; fetch confirmed the same remote tip. An independent clone is used for all work. No user files are reset.

Read AGENTS, README, index/status and AA/AE/AF/AH/AI/AJ conclusions before design. Stop repeating rejected seed/budget/weight searches. Audit official FAR-LIO at a pinned SHA and primary literature. Commit only original research and public aggregate history.

| Experiment | Prescribed stimulus | Acceptance / falsification |
|---|---|---|
| 1 | Same existing inertial predictor, 100 synthetic steps; normal NDT, +8m Z, +10/-5m XY and .4rad yaw; GNSS absent then position recovery | Protected local trajectory must equal no-map control; global output must expose wrong poses; position recovery must not imply yaw recovery |
| 2 | Known correlated Gaussian, full/Schmidt/projected observations | Compare information-form reference and joint-error covariance; scalar analytic case; symmetry/PSD, retained cross terms, protected mean and variance; information loss explicit |
| 3 | Infinite plane, orthogonal planes, three planes, missing two surfaces; repeated Gaussian cells | rank3/5/6/3; equal-cost separated peaks with positive local curvature must remain ambiguous |
| 4 | Normal/partial/ambiguous/stale/missing/nonconverged/rejected inputs; recovery and repeated input | Received != accepted; same scan consumed once; distinguish source and algorithm states |

7 new tests plus existing public suite. Fixed deterministic synthetic parameters; no fitting to real anomalies. All outcomes including counterexamples are retained. These tests establish algebra and containment, not calibration, CUDA-NDT behavior or real localization accuracy.
