# AV final decision — implemented and actually tested; full repair FAILED

**Do not promote this candidate to the research default or production.** Keep legacyAR/AS behavior. V3 completes a genuine1468-frame single lap, improves the two difficult right corners under the fixed engineering reference, but fails the frozen normal-region/global-XY screen. Continuous three-lap attempts also fail the research runtime's processing-age/history coverage. All failed designs and original results remain.

## Required answers

|Question|Evidence and decision|
|---|---|
|1. Why does legacy trust prediction?|VERIFIED under its model: AU fullP/R/H yields about0.10 lateral gain; covariance repeatedly contracts under the assumed independent likelihoods. One source-supported issue is carrying repeated-queue covariance compensation into single-use packet updates. This explains information accounting, not all physical bias.|
|2. Is high prior confidence physically justified?|NOT_VALIDATED. Arithmetic is correct, physical noise/cross-cloud correlation and estimator consistency are not calibrated. No evidence authorizes unconditionalP inflation, slip constraint or a new Bias model.|
|3. Is FitnessR matched to actual NDT quality?|NOT_VALIDATED statistically. Its smoothing factor has a traceable queue meaning, whereas its base score-to-noise and directional fidelity remain engineering heuristics. Native is not substituted without qualification.|
|4. What changed?|Opt-in packet likelihood; local geometry/three own-seed hypotheses; formally projected common constraints; full15D robust scale-mixture mean/total covariance; explicit bounded-history runtime adapter. Original propagation/wheel/Q/defaults/CUDA optimizer unchanged.|
|5. Why not directly shrinkR?|The removed factor compensates repeated use absent from research, proven by queue/source and information-budget identity. V3 separately marginalizes a declared finite-variance latent precision instead of selecting a GNSS-optimal multiplier. This is still an uncalibrated candidate likelihood, not proof of trueR.|
|6. How distinguish matching quality?|Keep convergence, support/rank, competing local basins, residual compatibility and posterior precision separate. Ambiguous combinations are withheld; common combinations may update. A shared wrong basin can pass; no certified correct-peak detector or calibrated probability is claimed.|
|7. How strengthen useful correction?|Restore unique-packet information and retain common measured combinations instead of whole-pose rejection. Robust residual likelihood avoids V2's hard-gate lockout. Right3/9 effective lateral gain rises to0.153/0.148; posterior history/residuals genuinely change.|
|8. Is full state/P reasonable?|VERIFIED numerical210 tests, independent mixture integration, actual full-state reconstruction, PSD and original SO3/lever/delay conventions. Statistical consistency NOT_VALIDATED; no NEES/NIS pass or true covariance claim.|
|9. Is feedback real?|VERIFIED: raw threeLiDAR→own Deskew→own prediction/seed→actual CUDA→historical-time update→reprop.1467/1468 Source hashes change; frozen-observation replay is not used as closed-loop evidence.|
|10. Did right3/9 improve without other degradation?|CONDITIONAL right3/9 P950.674→0.495m and0.668→0.552m, but normal segments degrade and fullXY0.577→0.591m at75ms. Full repair FAILED. OldEKF's right3/9 remain smaller under this convention.|
|11. Is long operation stable?|Full-P arithmetic/actual matching remains bounded through3458 continuous accepted frames; no independent state resets. Runtime/backlog fails before4134. Three-lap continuous qualification FAILED, not a successful long-run algorithm qualification.|
|12. What is verified?|Source information-use semantics, numerical model, exact legacy40 reproduction, actual lap execution and conditional comparisons. Physical GNSS/P/R/correct-peak/height/tyre safety remain unqualified.|
|13. Next research default?|NO. Preserve as an explicit experimental branch/candidate. Do not replace the frozen original. More gain alone is demonstrably insufficient for a general fix.|
|14. Production deployment?|NO-GO. Reference delay/installation/datum, statistical/correlation model, global peak integrity, accurate vehicle boundary qualification and actual ROS20Hz/100Hz runtime all remain blockers.|

## What the failure teaches

The packet semantics correction has independent source evidence, yet B0 alone improves difficult corners while worsening others. V1/V2 fail from observation starvation; V3 addresses those specific failure mechanisms, not a swept R/Q factor. Its full-state moment calculation is correct, but a robust likelihood does not fix every model mismatch or distinguish every correct basin. Smaller posterior uncertainty after stronger updates is not necessarily more honest uncertainty.

The continuous held-out-in-time lap2/third-prefix raw-engineering-reference XY differences also worsen. This rejects promotion on a single favorable corner metric. No further GNSS-driven covariance/precision-prior/geometry threshold search was attempted. Both gain limitation and reference/model/correlation uncertainties remain relevant; a unique physical cause of right-corner absolute error is UNKNOWN.

Next targeted work: first bound/chunk the runtime's retained inputs/ledger and measure reprop/allocation/GC before retrying continuous3laps. Separately collect independent time/installation/map-reference evidence and residual dependence across unique clouds, then assess a proper temporal/source-error model on held-out runs. These are specific blockers to a stronger information model; they do not justify scalingP/R until a preferred RMSE appears. Existing map/odom isolation remains an independent containment option, not a demonstrated precision fix.

## Git decision

Four logical research commits preserve design, implementation, actual feedback/failures and final reporting. Push the independent branch normally. As candidate acceptance failed and unresolved runtime failure remains, **do not fast-forwardmain** under this task's qualification condition. Main remains the frozen c647ac3224adee775d30dc6f9553c9b83612c060 if remote verification confirms it. This is a qualification decision, not missing authentication or a claimed merge. No production/default behavior changed.
