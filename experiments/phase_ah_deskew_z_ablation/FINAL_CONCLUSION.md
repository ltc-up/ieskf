# Phase AH conclusion

**Deskew compensation changes frozen geometry requests, but it does not by itself remove the unqualified Z covariance contraction or full-state velocity amplification.** Five representative events/6 predefined roles, same raw scans/motion/extrinsics/time, exact A reproduction, unchanged full6D CUDA-NDT and old Geometry-IESKF were evaluated. No continuous replay.

1. Relative world-Z translation: corrected B removes ONLY the kernel's base-origin relative Z term, keeping full rotation/LiDAR lever and fixed reference. NDT Z_B-Z_A=+3.73…+42.36mm in5 cases.
2. RP compensation: C preserves translation and yaw, freezes reference RP by a valid relative rotation. NDT Z_C-Z_A=−18.12…+9.50mm. Raw point P99 Z differences can exceed1m, without equivalent matched-Z displacement.
3. B has the larger absolute NDT-Z sensitivity in4/5 cases; RP dominates the largest-RP case. This sample does not establish a universal ranking. Failed fixedXY/yaw controls prevent a precise direct-Z vs XY/yaw-coupling partition.
4. Full Deskew A still proposes a0.936m first geometry Z correction, and later large velocity proposals occur; these are residual/update demands, not proven true height jumps. Nonadjacent frozen matched heights cannot establish a jump rate or accuracy.
5. E2 raw dvZ proposals A/B/C=4.809/5.117/4.862m/s are ALL rejected, accepted correction0. E3 legacy isolated accepted dvZ=1.082/1.211/1.074m/s; only baseline A corresponds to a historical carried trajectory. No B/C continuous or published state exists. Strict submission0/15.
6. Complete15D signed conditional decomposition attributes E2 chiefly to Z with meaningful XY/attitude cancellation; shared frozen prior supplies the amplification. Deskew changes requests, but all variants still collapse first sigmaZ1.077m→0.016/0.017/0.025m at MAX_ITERATIONS. A frontend-only explanation is insufficient for this historical posterior behavior; true statistical overconfidence remains uncalibrated.
7. Evidence supports revisiting nonconverged state/information commitment, NOT changing Z weights, velocity/Q/R, or suppressing cross-P. Existing strict-gate reject-all failure is not proposed as a solution. No independent true pose/height or calibrated covariance available.
8. Next priority: reliable geometric convergence and trustworthy posterior submission while retaining LiDAR constraint, with the current frontend fixed. Do not recommend B/C for production or interpret lower fitness/smoother velocity as localization improvement. Qualified constrained NDT diagnostic remains missing; do not add solver complexity/parameter search to force it.

|Qualification|Status|
|---|---|
|FROZEN_RAW_INPUT_AND_A_PARITY|PASS|
|CORRECTED_A_B_C_OPERATOR_EQUIVALENCE|PASS (5 new tests and recorded GPU/CPU parity)|
|FULL6D_NDT_PAIRED_SENSITIVITY|PASS within selected frozen cohort|
|FIXED_XY_YAW_NDT_CONTROL|NOT_VALIDATED (0 qualified; corrected B control not executed)|
|FROZEN15D_UPDATE_PARITY_AND_DECOMPOSITION|PASS numerically|
|QUALIFIED_GEOMETRY_STATE_P_SUBMISSION|FAIL (0/15 strict certificates)|
|CONTINUOUS_ABLATION / INDEPENDENT_ACCURACY|NOT_VALIDATED|
|STATISTICAL_CONSISTENCY / PRODUCTION_CANDIDATE|NOT_VALIDATED|

Production code/config/map/data/Git state and historical research remain unchanged. Only reviewed independent code, synthetic checks and deidentified aggregates are public; original kernel/SDK adaptation, raw points, full poses/P and detailed records remain local ignored. All mistakes and failed diagnostic attempts are retained, not replaced by best intermediate fits.
