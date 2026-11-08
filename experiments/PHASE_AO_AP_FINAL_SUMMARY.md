# Phase AO–AP final research summary
Baseline:b3de66aa7509b308bf73379892453743fab88676. AO:3667c9c6da84b82c1d9e4676b38436ed9f8f6da1. AP is the enclosing second phase commit (resolve with Git history; embedding a commit's own SHA in its hashed content is not possible). Two independently reviewed commits; remote SHA verification is performed after each push and after the final fast-forward. No production implementation was changed; the pre-existing production YAML edit is preserved.

**AO:NO-GO. AP:NO-GO for robustness acceptance; limited diagnosis/injection completed.** Executable research diagnostic prototype:CONDITIONAL. Newly connected complete ROS2 causal shadow chain:NO-GO. Production replacement:NO-GO. Neither synthetic tests nor safe rejection can supply missing localization availability or independent accuracy.

## Normal conditions
| Required answer | Actual finding |
|---|---|
|1. Real complete lap? | No qualified complete causal NDT/multisource lap. Existing local15D ran long raw inputs:loop7 144.86s,loop44 917.07s,loop8 2455.68s, stopping on real IMU-gap/end-of-stream gates. A prior73.4345s GNSS-derived lap window is fully covered by7344 local outputs, but local Z spans3012m and no trusted global output. |
|2. Global availability? |0% trusted coverage on executed manager prefixes:14486,91707,245568 output slots respectively. No anchor exists. Not a successful all-rejection localization system. |
|3. Independent accuracy? |NOT_VALIDATED. Derived “truth” CSV uses same GNSS and75ms correction; receiver/time/lever/datum/statistics not established. |
|4. XY/Yaw vs production? |No qualified RMSE/P95/precision improvement. Historical production poses lack same-runtime map/enable/acceptance reconstruction and are not truth. |
|5. Z/velocity reliable? |No. loop44 Z span70.720km,max wheel dZ150.939m;loop8 Z span310.906km,max wheel dZ597.146m. These are uncalibrated model behaviors, not independently measured vehicle errors. Full covariance/correlations retained. |
|6. GNSS fusion? |NOT_QUALIFIED. Fix8 is neither declared RTK Fixed nor invalid from old enums. Raw LLA is never treated as map-base pose; no invented R or yaw. |
|7. Qualified NDT candidates? |0 for these newly processed long-record public-pose streams. Convergence, Source/seed/map/reference/backend provenance missing. No new causal full-lap Deskew/CUDA matching chain achieved. Historical AM fits remain separately labeled. |
|8. Map/Odom isolation? |Executed global code has read-only local trace and copied poses; hashes unchanged. This isolates permitted writes, does not fix local vertical failure or guarantee correct global pose. |

## Abnormal conditions
| Required answer | Actual finding |
|---|---|
|9. Low-fitness wrong peak found? |Two historical AM multimode groups retained; no true/wrong labels, so real wrong-peak recognition NOT_VALIDATED. Explicit supplied ambiguity is preserved. |
|10. Map holes identified? |Injected20m hole removes3158 Target points; first support ratio drops3.2percentage points but rank6/VALID_CANDIDATE persists. Automatic partial-hole detection did not pass. |
|11. GNSS removes ambiguity? |Not demonstrated with real qualified GNSS. Unqualified GNSS leaves competing modes unresolved. |
|12. GNSS failure degradation? |600ms receipt injection gets STALE; dropout remains unavailable. Control already has no anchor, so no qualified COASTING/recovery sequence is established. |
|13. Partial LiDAR? |All requested single/pair/dropout raw frontend combinations NOT_VALIDATED. No random fused-point deletion masquerades as a LiDAR outage. |
|14. High dynamics stable? |Raw wheel max70.155m/s and gyro norm max0.951rad/s observed; stable accurate localization not established, local height fails. |
|15. Wrong matches alter local velocity/Bias? |All nine global fault scenarios retain the same complete local trace SHA. No local writable state/P handle. This tests isolation, not real wrong-peak detection. |
|16. Recovery valid? |Real global recovery NOT_QUALIFIED:zero anchors throughout. Existing synthetic recovery remains synthetic. First wrong shadow probe fails NO_BRACKET at startup; not silently moved. |

## Engineering qualification
| Required answer | Actual finding |
|---|---|
|17. Independent real evidence? |Raw stream identities/continuity, actual long-input local outputs/full P, recorded receipt timing and interface behavior are real. No independently qualified localization accuracy. |
|18. Synthetic/injection only? |Map deletion, GNSS delay/dropout/outlier, NDT wrong/duplicate/delay/two-mode/absence are artificial on real records. Wrong bracket-valid global pose isolation has synthetic unit evidence only. |
|19.20Hz? |NOT_VALIDATED. New offline core/geometry distributions recorded; no fresh complete Deskew/CUDA/ROS2 end-to-end timing. Module P95 values are not summed. |
|20. Production shadow deployment? |New full causal stack admission NO-GO. Existing prior isolated ROS transport is not this task's qualified online chain. |
|21. Replace original output? |NO-GO. No normal-lap global functionality, independent accuracy, local-state health or deadline qualification. |
|22. Exact blockers? |Local sensor/reference/gravity/initialization/time/model qualification and vertical observability; zero wheel Headers in primary; input gaps/tails; complete NDT candidate/map/Source/seed/provenance export; causal raw Deskew/frontend generation; GNSS hardware protocol/clock/lever/map/noise calibration; independent truth/vehicle acceptance limits; ROS2 callback/backlog measurement and local-health publication gate. |

## Timing audit result
Production6-state augmented lag filter, pose smoothing and actual timer-dt behavior are explicitly distinguished from15D replay and map history. Same2292 real pose pairs: latest-arrival pairing differs from Header pairing by up to38.808m in correction translation; historical same-time pairing agrees. This is not vehicle position error.

Independent bounded checkpoint/repropagation over43441 real IMU/wheel events(144.85s) exactly reproduces final Header-order state and all225 P entries after handling one temporarily missing IMU. Initial failed attempt is retained. Maximum153 buffered events; no repeated independent information. Actual node callback ordering, online startup, production smoothing equivalence and ROS deadlines remain unvalidated. The initial state is shared for comparison; this does not certify online initialization.

## Input and delivery identities
| Input | Raw file SHA256 |
|---|---|
|loop35 raw SQLite|df113e7df75057354a7b3bcd8f613946bab6551db0860f99b9ad23480e8f395c|
|loop44 raw SQLite|0be29d090557accc27581007b45660cb5b0063dbcbdd3f3b798e33ff64c1db14|
|original loop7 MCAP|39a58bb1e6f98ce8fdff9b04945cb040ddb197449f6f29edac40c4a2cb727f22|
|original loop8 MCAP|3f2a553ef79528fc4802112a71a891cc5d8be1d1a76dd0c2190b3c4bab6020e5|

Full raw input/extrinsic/trajectory payloads remain in ignored private storage; public manifests record hashes and aggregate results only. AO129 tests pass/0 fail; AP134 pass/0 fail (cumulative, not263 unique tests). These counts do not include engineering qualification failures. Three raw local attempts stop on missing IMU; primary wheel fusion is unqualified; first delay attempt and ineffective real wrong-shadow probe remain recorded.

Borrowed FAR-LIO design remains source health/timeout/accepted-vs-received separation, adapted in prior AN; no FAR-LIO state model/GICP frontend was imported and no new claim of full reuse is made. Original point-plane Geometry-IESKF is not reintroduced.

First next change: independently qualify and instrument the existing local15D sensor/time/reference path, and add an explicit local-state eligibility input to global publication. In parallel, add a read-only NDT candidate export carrying actual Source Header/reference/map/extrinsic/seed/correlation/convergence/backend/LiDAR identities, with UNKNOWN diagnostics preserved. Then connect the checkpoint scheduler and raw frontend in an isolated namespace with no production TF ownership, measure actual callback/queue delays, and rerun a preselected complete lap with an independent reference and prior acceptance standards. Feature-flag disable is the rollback; global corrections must not reset local odom/velocity/Bias/P.

[AO report](phase_ao_full_lap_validation/FINAL_CONCLUSION.md) · [AP report](phase_ap_robustness_validation/FINAL_CONCLUSION.md) · [timing audit](phase_ao_full_lap_validation/ENGINEERING_TIMING_AUDIT.md) · [AO manifest](phase_ao_full_lap_validation/manifest.json) · [AP manifest](phase_ap_robustness_validation/manifest.json).
