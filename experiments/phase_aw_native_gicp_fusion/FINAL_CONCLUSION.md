# AW final route: retain CUDA-NDT default; GICP runnable, not a proven replacement

**Completed:** source/stash recovery, existingNative audit,1468 real same-input matcher pairs with bounded probes, GICP own-state1468 lap and4134 continuous3laps, same-fixedR NDT actual causal control, historicalNative reevaluation, full-lap timing/resource replay,195 tests and preserved failures. **Not implemented:** a new directionalR/adaptive fusion candidateD, because actual results do not justify that statistical model. No invented adaptive gain or default promotion.

## 18 required answers

|Question|Evidence and qualification|
|---|---|
|1. Source branch/commit/stash?|VERIFIED production `ndt_test` aa50e354; relevant untouched stash c964ce45. Research main c647ac3 in a separate worktree; AV b143e88 preserved.|
|2. Reused code?|VERIFIED existing MIT small_gicp fa0cfc98 PointCloud/KdTree/TBB covariance/GICPFactor/LM/rightSE3, prior registered settings and adapter design; existing15D/Deskew/delay/native_pose_R. New independent protocol glue, not another GICP solver or copied production package.|
|3. Native versusH?|VERIFIED source formula: averaged best-voxel inlier GN, clippedκ, eigen floor and scale; not inverse exact DIRECT7 scoreH. Logged rounded-poseκ limits exact absoluteGN recovery; shape/coupling still usable.|
|4. Native directions effective?|VERIFIED conditional objective/chart diagnostics, shape often similar to negative scoreH; actual error prediction/calibration NOT_VALIDATED.|
|5. Same right-bend poses?|CONDITIONAL: raw GICP/NDT are often close but not identical; selected GICP probe solutions differ up to0.152m in3 and0.319m in5. No independent correct-peak labels.|
|6. Closer to engineering reference?|Common1141 raw-valid75ms samples: NDT/GICP XY≈0.608/0.610m overall. GICP raw right3/9 improve but normal5 worsens. Neither uniformly wins. All0/75/150ms retained.|
|7. Multi-peak/degeneracy?|VERIFIED local initialization/association sensitivity and anisotropic information. No rank-zero map-hole or certified wrong-peak dataset in this normal lap. Stable localH does not separate nearby solutions.|
|8. Own GICP real loop completed?|VERIFIED1468/1468 with1401 updates and4134/4134 with3956 updates; same initial condition, no per-lap resets/cache extension. CPU GICP, CUDA Deskew.|
|9. Right3/9 improved in filter?|FAILED fixed75ms repair criterion: posterior lateralP950.833/0.737m versus original0.674/0.668m. Better raw candidates do not alone solve feedback.|
|10. Normal segments?|FAILED no-regression criterion: C lateralP95 worsens in all fixed normal segments versus originalA under this reference.|
|11. FullXY improved?|FAILED: C0.596m versus A0.577m; NativeB0.631m. With identical fixedR, GICP0.596 beats NDT0.612m, showing a conditional backend contribution rather than default superiority.|
|12. FullH direction agreement?|Broad normalized shape median cosine0.900, weakest-axis cosine0.446 and minimum lowest2D-subspace cosine median0.914 in a common seed-axis chart. Nearby eigenvalues can rotate the individual weak basis. Objective representations differ; not interchangeable covariance.|
|13. Non-diagonal value?|VERIFIED full-coupled conditional geometry; marginal versus conditional direction energy differs, right9 lateral–Yaw coupling≈0.606 forGICP. Preserving terms is mathematically necessary; actual calibrated fusion benefit NOT_VALIDATED.|
|14. Enough evidence for newR?|NO. Geometry/error Spearman≈+0.049/−0.056, unqualified physical reference, surrogate point covariance and cross-frame correlation prevent a justified error-scale mapping.|
|15. Adaptive fusion benefit?|NOT_VALIDATED / NOT_IMPLEMENTED. D is explicitly withheld, not a successful unused plan. Fixed matched-R control is an implemented diagnostic, not an adaptive model.|
|16.20Hz?|Offline3lap section P95≈26.97ms, max109.65ms; strict50ms deadline FAILED. Max modeled observationage287.33ms, no500ms cache loss. ROS20Hz/100Hz NOT_VALIDATED.|
|17. CUDA GICP worthwhile now?|NOT_SUPPORTED. CPU matching P95≈8.36ms in own-state3lap workload and no broad accuracy gain; a GPU port is not the demonstrated bottleneck or justified precision remedy.|
|18. Next route?|Retain NDT original default. Keep GICP as an experimental matcher/control, prioritize independently qualified time/reference and fusion/source-noise/temporal information modelling, not new SLAM or blind Native/Hessian weighting.|

## Actual boundary of progress

Runtime completion improved meaningfully: journal streaming/static heap lifecycle preserves original500ms cache and actual elapsed processing; legacy40 matches exactly, fully instrumentedGICP1468 agrees≈1e−13. This is not certified ROS scheduling or removal of all long tails. No original/default/production equation, wheel protocol, Q, Bias clamp or TF publisher changed. Original user branch/stashes/untracked assets remain.

GICP fullP stays numericallypositive (minimum≈5.00e−7), Z span≈10.10m and state/Bias remain finite through3laps, without cross-term removal/frozenZ. These do not prove physical covariance/height accuracy. Lag1 lateral innovation correlation≈0.971 is a descriptive prediction/measurement signal, not directly measurement-noise correlation. It motivates qualification, not a fittedAR coefficient or unconditionalP inflation.

HistoricalGNSS, physical antenna delay/installation, map-height datum and visual vehicle outline are CONDITIONAL. ExistingCSV disappeared externally after reference snapshots had been computed; exact cached native-time reference arrays/masks are retained/hashed. No fitted replacement/time selection was introduced. No absolute localization, tyre safety, correct-peak detection accuracy, statistical covariance coverage or production readiness is claimed.

## Git/default decision

Source audit, actual implementation/closed loops, and final decision/reproduction are separate reviewed commits on `research/phase-aw-native-gicp`. Because originalA is not improved and matching risks remain unexplained, keep the independent branch and do not merge/promote it to main under the user's acceptance rule. No authentication/merge success is fabricated; final remote SHA is verified at delivery. A future isolated runtime-only review is possible, but is not silently merged here.
