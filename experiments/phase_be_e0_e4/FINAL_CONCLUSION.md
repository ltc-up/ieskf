# Phase BE-E0–E4 final conclusion

本轮已实际实现并执行E0～E4研究，但系统验收未通过：双分支GMF没有证明模式选择收益，两个三圈长测均失败；正常段8的光学方案退化仍然存在。全部结果和失败已保留，生产代码未修改。

**Implemented and experimentally exercised; system acceptance FAILED / production NO-GO.** Do not equate this research delivery with a robust racing localization system. The original production six-state EKF, Q/R, NDT defaults, converter, static TF and maps are unchanged. Research code commit: `00de60bd9b7e1d2ee9c475e82e33aa66923f85e5`. The separate report/publication commit contains these artifacts.

| Requirement | Evidence | Qualification |
|---|---|---|
|15D independent branches|own mean/fullP, inputs, deskew, updates and ledgers;218 tests pass|implemented; statistical calibration unproven|
|Directional observation|normalized coupled charts; synthetic weak-axis retention; all real frames remain rank3|normal5/8 proxy-good updates retained; real degeneracy benefit NOT ESTABLISHED|
|Optical/lateral motion|actual raw XY/lever model; single/triple feedback completed|conditional gains, normal8 regression and reference-lag reversal; no new sideslip model accepted|
|NDT/GICP complementarity|1468 same-input pairs plus historical12632;1073 separated candidates|no strict proxy-good/bad rescue sample at75ms; replacement NO-GO|
|Dual GMF single lap|1468frames, two branches, zero switches;0.577m proxy RMSE|no selection gain; P95=47.87ms/max179.88ms/44overruns|
|GMF three laps|4052/4134 before input-horizon guard|FAILED, not a complete three-lap result|
|GMF+recovery|single lap complete; exact retained-event rollback tests; five revisions/branch|P95=65.90ms; three laps4035/4134, FAILED|
|Control continuity|versioned map→odom/path contract, separate motion chain|controller and joint covariance NOT VALIDATED; high-speed automatic relocation NO-GO|
|Production six-state|actual queue-header test and source audit|compatibility findings only; no production accuracy/timing claim|

The final controlled15D baseline has75ms XY reference RMSE0.579m(single)/0.571m(three laps). The matched35-iteration/held-out control is0.577/0.571m. OpticalXY is0.517/0.517m, but normal8 lateralP95 worsens0.344→0.427m(single) and0.411→0.436m(three laps). This repeats the important conditional benefit/tradeoff rather than resolving it. GNSS position lags0/75/150ms and a separate delivered-current-state evaluation are all retained. GNSS is an imperfect offline proxy, not the only correctness authority; shared-map validation is also correlated. True mode-recognition latency and true false-admission/rejection rates remain UNKNOWN.

What is useful now: terminal observation identity, no resurrection of rejected data, bounded full-state/P rollback, explicit nonrevocable history, independent branch containers, and physical chart diagnostics. What is not established: an automatic verdict identifying wrong converged NDT modes, a calibrated mixture likelihood, real-data weak-axis suppression benefit, an independent physical sideslip improvement, or safe controller recovery. A positive local Hessian and finite converged fit still do not certify the right branch.

small_gicp may remain an opt-in Shadow source, but neither a replacement nor a second independent probability vote is justified. Keep the dual-bank prototype for research; do not port it as production behavior or add particles/branches to hide the evidence and latency problem. A future implementation needs bounded compiled replay cost and independently validated temporal evidence before a new full-loop qualification. Forensic IO, Python execution and a nearly-full NTFS volume are part of the measured prototype workload; its results do not bound a future compiled implementation.

The current production queue needs, at minimum, unique IDs/final status, source-aware retry/expiry, restorable mean/P plus raw-input journal, deterministic revision/replay, and coordinated seed/last-trusted/frame-version updates. Removing an old queue item does not undo an already fused update. Do not replace the six-state model with this15D code or copy its tuning wholesale.

GNSS remains solely under existing production switches. Research online events do not consume GNSS, and no FIX8 auto-enable, RAIM/FDE or converter modification is added. The legacy antenna-to-rear-axle defect is audited, not fixed. Frozen initial-pose ancestry is not independently certified GNSS-free. Current `aa50` lidar_ndt source/YAML selectsGLL3; historical `c647` OFF and an unverified deployed v1 path are distinguished. Exact production-service binary equivalence is not claimed.

All failures are retained, including startup syntax/test failures, earlier source-history/backlog problems and failed three-lap prefixes. Selected key earlier sources are byte-recovered against their captured SHA; original restricted inputs/services are not redistributed.509070 result files (455166849430bytes) are SHA256-indexed with no observed change during hashing;16812 raw/dependency paths and1469 frozen requests are separately identified. Numerical regression is not accuracy certification.

BE5 is queued only after this delivery: read-only actual-data-flow/Jacobian/covariance review for A/C and Legacy/Native/Native-Calibrated, with B excluded and approval required before BE5 code. No BE5 constraint, covariance retuning or optimizer change is included here.

See [accuracy](ACCURACY_COMPARISON.md), [performance](PERFORMANCE_COMPARISON.md), [E4](E4_DELAYED_FUSION_AND_RECOVERY.md), [production compatibility](PRODUCTION_6STATE_COMPATIBILITY.md), [reproduction](REPRODUCE.md), and the private storage layout supplied with the handoff.
