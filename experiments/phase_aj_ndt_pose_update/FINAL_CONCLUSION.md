# Phase AJ conclusion

The direct NDT XY/yaw observation interface is numerically valid and supports actual full15 state/P recursion, but this frozen engineering baseline does NOT stably preserve most LiDAR constraints. C commits66/284 (23.24%), compared with B's233 historical partial geometry acceptances. It has a5.50s gap and108 consecutive rejects. It bypasses the failed point-plane optimizer rather than repairing it.

## Answers

1. **Stable LiDAR retention?** Not demonstrated:66 actual submissions,129 NDT nonconvergences,87 guard failures,2 R failures. Some later recovery occurs, not sustained retention.
2. **Avoids B's internal optimization failure?** Structurally yes: no point-plane iteration or partial point-plane Hessian is submitted. Overall localization failure is not eliminated.
3. **Unexpected Z/vz via full P?** Yes:28 guard-rejected proposals have planar norm <=.5m but the full3D proposal is unsafe. L079 loses LiDAR despite small planar innovation because induced dZ is−.551m. Also, despite no LiDAR Z channel: max actual LiDAR |dZ|=.310943m, |dvz|=.160987m/s. A rejected −25.81784m/s proposal is not an actual jump. Further actual wheel-induced jumps reach4.13669m Z/.559931m/s vz through the unchanged full P.
4. **Without LiDAR Z?** C height spans10.047m and ends+2.491m relative to initial state; final sigma Z47.596m. Absolute height accuracy is unknown, but3D observability/behavior is clearly not solved.
5. **Actual state/P recursion?** Yes numerically:66 single Joseph/reset atomic commits,218 atomic holds;153 saved-prior rechecks reproduce the run to roundoff; all saved P stay SPD and all100Hz slots exist.
6. **Wrong NDT peaks?** Poor/converged matches and very large innovations/proposals occur; some are rejected, five high-fitness cases submit. Independent truth is absent, so no true wrong-peak rate or localization accuracy claim is possible. Matching/feedback stability remains unqualified.
7. **20Hz?** Typical measured offline costs fit50ms, but3 callback overruns and absent full ROS/Deskew scheduling mean no end-to-end qualification. No timing retuning or rerun.
8. **Useful engineering baseline?** Keep the independent interface and this negative closed-loop result as a comparison, not a validated3D solution. Next priority is legitimate vertical observability plus evidence-based matching/feedback and safety-semantics analysis, preserving the predictor until causal evidence supports any change. Do not return to blind point-plane iteration/weight searches.

| Qualification | Status |
|---|---|
| FROZEN_INPUT_AND_INITIAL_PRIOR_IDENTITY | PASS |
| CAUSAL_C_NDT_SEED_IDENTITY | PASS,284 exact backend seeds |
| XY_YAW_OBSERVATION_NUMERIC | PASS,11 focused tests |
| REAL_STATE_P_ATOMIC_RECURSION | PASS, numerical/model-conditional |
| IMU_WHEEL_OUTPUT_COMPLETENESS | PASS, offline grid |
| MOST_LIDAR_CONSTRAINTS_RETAINED | FAIL |
| FULL3D_HEIGHT_STABILITY | FAIL |
| NDT_PEAK_CORRECTNESS | NOT_VALIDATED |
| STATISTICAL_CONSISTENCY | NOT_VALIDATED |
| INDEPENDENT_ACCURACY | NOT_VALIDATED |
| END_TO_END_20HZ | NOT_VALIDATED;3 measured overruns |
| LONGER_UNCHANGED_REPLAY_READY | FAIL |
| PRODUCTION_CANDIDATE | NOT_VALIDATED |

Public material contains independent code, synthetic tests and self-contained deidentified statistics. Full Sources, map, SDK adapter, vehicle states/P, trajectories and extrinsics remain ignored locally. Production files/Git and all historical evidence are preserved. Neither a smoother trace nor agreement with B is claimed as accuracy improvement.
