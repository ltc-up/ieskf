# Phase AQ final summary

Baseline d6b5441a7a12d8f2f282f75399fe8bec26c72831. AQ1 e442d915d2c04cc8f8a3f3db05bbe49d04afb814; AQ2 4a57f4d1c8789f5ba550d23e7982146487bcf078; AQ3 is enclosing commit.153 tests pass,0 fail;19 added across AQ (6+5+8). Actual-data qualification is separate from unit-test success.

## Motion model: questions1–6

1. CONDITIONALLY_VERIFIED: vertical propagation drift precedes huge wheel corrections. Velocity-only observations cannot anchor absolute position; attitude/bias/gravity/noise physical calibration is unresolved. Original8 all-zero wheel despite moving Kistler is an input protocol failure. User confirms newer eav24/eav25 transition: only newer loop10 wheel trusted, older eav24 use loop44. Do not infer all kilometre drift has one proven cause.
2. VERIFIED preserved AQ1 aggregates:597.145845m = Kz−2406.870662s × innovation−.248100513m/s. Kz blocks velocity−1388.321452, attitude−1018.773653, gyroBias+.224443s. Pzz15209304.961m². Untrusted long-horizon cross covariance amplifies a small innovation; protocol-mismatch channel is not valid wheel-performance evidence.
3. VERIFIED mathematical tests: wheel H and right-error propagation F/G pass finite differences with sensor axis/lever. Sampled-noise Qpp differs from continuous-white-noise discretization by25% locally; causation of597m not established.
4. NOT_VALIDATED physically: specific-force/unit conventions supported by publisher/sample audit; runtime axes, initial attitude/gravity/bias and noise calibration incomplete.
5. NOT_VALIDATED: speed-only Rw adjustment unjustified. Protocol qualification before speed/slip/turn/residual grouping; no weight inflation performed.
6. CONDITIONALLY_VERIFIED Kistler measured1D sensor-frame speed with lever/axis handling. No invented3D velocity or independent joint wheel fusion. Short ablation is not accuracy ranking.

## NDT closed loop: questions7–13

7. VERIFIED preserved AQ2 public execution: raw clouds→own15D history/Deskew→own prediction CUDA-NDT seed→historical external pose update→next prediction. Actual loose pose coupling, not point residual IESKF.
8. VERIFIED actual Z used in C XYZ/yaw:30/40 accepted over1.950s. B XY/yaw28/40; D direction-projected19/40; Map/Odom E does not update local state.
9. CONDITIONALLY_VERIFIED local normal geometry: support>=.94267, rank6 on40; not calibrated6D noise or verified map-height accuracy.
10. NOT_VALIDATED long-term height: short C Z span1.237m versus A4.364m at respective publication times is output behavior, not independent height error.
11. NOT_VALIDATED physical correctness of full-P velocity/bias map corrections. They genuinely occur and are logged; no Kz/P masking, fixedZ or vz clipping.
12. FAILED normal continuation: requested20s stops55/401 at2.700029s with32updates after.549962s no acceptance. Remaining346 NOT_EXECUTED. Low-fitness MAX_ITERATIONS not promoted.
13. NOT_VALIDATED full normal lap. This short closure is not a lap pass; preceding no-NDT drift is not failure of a complete loose-coupled architecture.

## Covariance: questions14–18

14. CONDITIONALLY_VERIFIED restored Native RxRyRz-to-right/yaw covariance transport mathematics. NOT_VALIDATED statistical covariance. Reported angular score-Hessian source derivative mismatch needs runtime-build corroboration; independent expression counterexample is tested. Native uses distinct first-derivative information.
15. VERIFIED diagnostic distinction: normalized geometry/subspace/support provides directional meaning fitness lacks; no eigenvalue-to-R conversion.
16. CONDITIONALLY_VERIFIED mechanism: holes, weak directions and wrong peaks need distinct evidence. Prior35-fit transcript records one0/7-converged group despite low fitness and no certified competing peaks; raw AQ3 artifacts now lost. No uniqueness/detection-rate claim.
17. NOT_VALIDATED statistical calibration: no independent map/time/lever/protocol-qualified reference, no real RMSE/NEES/NIS pass.
18. CONDITIONALLY_VERIFIED experimental Native candidate only. Prior transcript records29/40 short updates and stop56/401 at2.75s; raw logs lost, not reproducible qualification. No R approved for production; FAR-LIO health/config-floor design does not calibrate NDT.

## Engineering: questions19–22

19. VERIFIED structural distinction: full15D allows global position to constrain coupled motion/bias but bad peaks can corrupt them. Map/Odom isolates local feedback, without curing missing vertical information. AQ2 A/E native-reference p/R/P exact40/40 in preserved public summary.
20. NOT_VALIDATED superiority over production EKF: no independent controlled accuracy comparison.
21. Unverified premises: runtime sensor/initialization/noise contracts, GNSS fix/posu, map/geodetic/height alignment, antenna lever/time/independence, angular Hessian runtime provenance and peak/noise statistics. Replay IMU/wheel Header availability is not full ROS callback equivalence.
22. CONDITIONAL bounded normal debugging only. NO-GO full-lap acceptance, covariance-qualified integration or production replacement. First recover durable raw/adapter artifacts and rerun, explain convergence failures without tuning-to-pass, close physical/reference contracts, then freeze and extend normal windows.

## Evidence preservation failure and data scope

Environment replacement removed temporary research clone/private outputs before AQ3 commit. Pushed AQ1/AQ2 public commits recovered exactly; AQ3 core/tests rebuilt and rerun153/153. Prior AQ3 GPU logs and private persistent Deskew adapter were not recoverable. Their transcript aggregates are explicitly NOT present-day reproducible execution evidence. AQ1/AQ2 private-delivery promises are superseded by this disclosure; original bags/maps and old AO/AP extracted data survive, new bindings/adapter require authorized rebuilding. No invented raw records or synthetic substitute.

Real input was legacy loop44 bag SHA256 0be29d090557accc27581007b45660cb5b0063dbcbdd3f3b798e33ff64c1db14; Target0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968. AQ1 long no-NDT traces are causal audit only. AQ2 raw120cloud normal40 plus401-triplet20s extraction/prefix; not a lap. Historical899-frame covariance audit is separate input/Target. Prior multistart batches59–225ms are transcript-only; no20Hz/ROS timing PASS. No production changes or TF publication.

Reports: [AQ1](phase_aq1_motion_model_audit/FINAL_CONCLUSION.md), [AQ2](phase_aq2_ndt_15d_closed_loop/FINAL_CONCLUSION.md), [AQ3](phase_aq3_ndt_covariance/FINAL_CONCLUSION.md). Final Git SHA/remote status reported after normal commit/push verification, not recursively embedded in its own content.
