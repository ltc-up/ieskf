# Phase AI conclusion

NDT seeds do not solve first-frame Geometry-IESKF capture under the unchanged point-plane objective and engineering guards. All five pose seeds are pose-feasible, but conditional full-state completion violates bias guards in three. At the original four-loop budget neither initialization gives a strict solution (0/5 each). One predeclared 16-loop diagnostic on the two feasible seeds qualifies E5/NDT at loop 7; E1 remains unqualified from both starts.

Thus NDT can improve a local optimization start, but there is no all-frame or first-frame remedy. NDT fitness/convergence cannot certify the different point-plane MAP. E1's remaining association witness and optimality failures are direct evidence; attributing all failure solely to association or solely to iteration count would exceed the evidence. Bias-guard exclusion and objective/prior penalties are separate mechanisms.

The qualified E5 conditional P reduces sigma Z from 27.60 to 24.70mm and is numerically positive definite. This is not a calibrated posterior. No real state/P was committed, no candidate correction is a published/replayed state jump, and no continuous replay was run. Stop seed/budget expansion on this route; the first-frame objective/association and credible information problems remain.

| Qualification | Status |
|---|---|
| FROZEN_PRIOR_AND_INPUT_IDENTITY | PASS (backend scalar-rounding limit disclosed) |
| SEED_INTERFACE_NUMERIC | PASS, 6 new checks |
| NDT_POSE_GUARD_FEASIBILITY | PASS, 5/5 |
| CONDITIONAL_FULL_STATE_SEED_FEASIBILITY | CONDITIONAL, 2/5 |
| ORIGINAL_BUDGET_STRICT_GEOMETRY | FAIL, 0/5 both starts |
| EXTENDED_NDT_LOCAL_GEOMETRY | CONDITIONAL, E5 only; first frame fails |
| FIRST_FRAME_CAPTURE | FAIL |
| CONDITIONAL_POSTERIOR_NUMERIC | PASS, qualified E5 only |
| INFORMATION_TRUSTWORTHY | NOT_VALIDATED |
| REAL_STATE_P_ATOMIC_COMMIT | NOT_VALIDATED; zero performed |
| CONTINUOUS_REPLAY_ELIGIBLE | FAIL |
| INDEPENDENT_ACCURACY | NOT_VALIDATED |
| REALTIME_20HZ | NOT_VALIDATED |
| PRODUCTION_CANDIDATE | NOT_VALIDATED |

Next work should address the already documented first-frame geometry/information mismatch with evidence, not search additional seeds or loosen guards. The independent production localization pipeline is unchanged.

