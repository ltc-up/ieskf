# Phase AU — Prediction accumulation and NDT feedback

Frozen baseline ac8e87f5755805c23c0dae305794000939a82299. Three logical research commits; no production, AR/AS/AT algorithm, Q/R, NDT parameters, vehicle configuration or GNSS-fusion changes. Real original motion and1468 frozen CUDA-NDT transactions were used to recover full histories; **no new CUDA/Deskew rematching** was executed.

## Q1 — How the right-corner prior difference forms

**VERIFIED difference decomposition; physical error CONDITIONAL.** Section3's lead-in grows from .0058m prior−NDT at frame1298 to .3150m at entry1317. The entry value contains .2535m inherited previous-posterior discrepancy and .06145m new motion-minus-NDT displacement. At exit it falls to .0513m. Same-sign run starts1298 (64.900s from first scan); several larger relative increments occur before entry, not just inside the corner.

Section9's same-sign run begins822 (41.100s), before the frozen padding window; by padding849 prior−NDT is already .2890m. At entry869 .1385m contains .1235m inherited and .01506m new relative displacement; exit .0133m. The sign-run is a descriptive NDT discrepancy, **not** an independently labelled physical-error onset. GNSS-reference differences and axes can give other signs/trends; no reference was selected to favour a cause.

## Q2 — Evidence about yaw, velocity, Bias, wheels and time

**Nominal history VERIFIED; unique physical cause NOT_VALIDATED.** Section3 entry prior−NDT heading is1.450deg, reduced to1.336deg after NDT; estimated base lateral velocity averages+.978m/s in3 and+.483m/s in9. Wheel direct lateral base corrections average−.452/−.921mm per NDT interval, but wheel-induced velocity/attitude/Bias changes also propagate. These are estimated states and reference differences, not true slip, true yaw/Bias faults or permission to impose zero lateral velocity.

Inertial midpoint/F/G and wheel/pose Jacobian definitions remain the already audited equations. Recorded-envelope replay exactly matches every AR prior/seed/gain/delta and published mean/P/time. Canonical propagation and update displacement components close to5.52e−13m. This rules out a new replay/order mismatch in the diagnosed numerical run, not a physical sensor-clock offset or real ROS scheduling fault. Actual GNSS latency/installation/map reference and true Bias remain unqualified. No singular wheel or IMU event is proven to be the root physical fault.

## Q3 — Why only centimetres of correction

**VERIFIED mathematical mechanism under uncalibrated assumptions.** Mean body-lateral innovations−.152/−.119m in3/9 yield exact rear-axle updates−.01546/−.00955m. Full unit lateral gains are .1032/.1055. Projected prior standard-deviation scale is .0626/.0693m versus engineering R's .1858/.2029m. The current covariance assumptions favour prediction, so partial correction is expected. AT's track-normal−.01587/−.01220m uses a different axis and is retained separately.

## Q4 — P, R, H, residuals or coupling?

**VERIFIED conditional decomposition; physical P/R correctness NOT_VALIDATED.** Full15D H includes the rotating rear-axle lever. Independent K/delta/Joseph/reset errors are below7e−16. Position, attitude-residual and exact lever corrections are separated; mixed real residuals explain why realized ratios need not equal unit gains. The close scalar/full responses compare observation-channel mixing conditional on **full HPHᵀ**, not a filter with removed state correlations. Signed pose variance terms are preserved and reported. Global P×2 and R×.5 produce identical gains in the frozen-state experiment, so gain alone cannot identify P underconfidence versus R overconfidence. No covariance is statistically calibrated here.

## Q5 — Does propagation erase the previous correction?

**No universal erasure: VERIFIED for selected fixed-observation mechanisms; general closed-loop claim NOT_VALIDATED.** Factual section3 has many opposing relative-motion/correction intervals (64/89), but section9 net new NDT-referenced difference is small. Current-axis transport must be included. With one selected NDT update omitted, its original lateral influence persists to the next prior at106.6%/104.3% in right3/9; it is not overwritten. Future frozen measurements then alter the influence. Single selected wheel skips have smaller next-prior effects, without ruling out aggregate indirect wheel history.

These250ms-cutoff branches preserve integration splits and full covariance. Their baseline means/P match original event histories exactly. Own subsequent seeds change; actual CUDA results under those seeds are unknown because rematching was not run. No fixed-observation branch is called a new real closed loop or full-lap improvement.

## Q6 — Is modifying the model or Q/R justified?

**NOT_VALIDATED; insufficient evidence for a physical-model or production-weight change.** The strongest supported mechanism is inherited discrepancy plus modest current observation response, with additional NDT-referenced lead-in motion disagreement in section3. Physical yaw/velocity/Bias error, input time misalignment, geometry bias and observation statistical uncertainty cannot yet be uniquely separated. GNSS engineering time hypotheses can reverse accuracy ranking.

The next targeted experiment should retain these exact lead-in windows and obtain independently timed/qualified heading and motion observations, then compare a separately declared candidate through actual Deskew/CUDA feedback. A controlled paired gyro/velocity/time-contract test is more justified than blindly reducing R. Any future covariance change needs separately frozen assumptions and reference qualification; it cannot be chosen by minimizing this GNSS score.

## Delivery and qualification

183 unit tests passed,0 failed (12 new relative to AT171). All1468 frame-equivalence comparisons pass exactly; original AR/AS/AT used inputs and transaction SHA checks pass. Full state/P/input histories, repropagation revisions, canonical history, corner uncertainty/velocity/Bias/gain plots and9 short-window baseline/intervention branches are private. Public reports/JSON/figures contain only nonreconstructive aggregates.

Research mechanism diagnosis: VERIFIED numerics / CONDITIONAL physical interpretation. New CUDA causal execution: NOT_EXECUTED. Independent true accuracy, calibrated covariance, physical root cause, ROS2 realtime and production qualification: NOT_VALIDATED. No finding supports changing the frozen production configuration in this phase.
