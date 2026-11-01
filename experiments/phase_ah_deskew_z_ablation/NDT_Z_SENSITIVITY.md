# NDT height sensitivity

Primary results use unchanged CUDA-NDT DIRECT7 binary/library, full target hash, recorded active region/crop, same native-time prediction pose per event, same resolution1.5/max35/step0.2/epsilon0.01/GLL3/q50 and health/fitness evaluation. Neither initial pose nor target follows an A/B/C result.15 PRIMARY full6D fits; an earlier excluded B-definition error adds5 obsolete fits, retained privately and never mixed. All15 primary fits report OBJECTIVE_CONVERGED; that is the backend's stopping label, not global-optimality or accuracy proof.

|Event|Z_B-Z_A mm|Z_C-Z_A mm|PCL fitness A/B/C|Iterations A/B/C|
|---|---:|---:|---|---|
|E1|33.678|9.499|0.131647/0.131419/0.131387|26/25/26|
|E2|34.937|4.936|0.123018/0.123944/0.123814|22/20/20|
|E3|42.360|-5.257|0.120921/0.123502/0.121053|24/21/24|
|E4|12.922|-18.118|0.118461/0.118610/0.116774|17/18/14|
|E5|3.733|-1.307|0.087274/0.087590/0.087123|10/10/12|

Across these5 cases, B shifts NDT Z+3.73…+42.36mm; C shifts−18.12…+9.50mm. B has larger absolute matched-Z effect in4/5 cases; C is larger at E4. This does not mean B is physically worse/better, or that this holds outside the frozen cohort. At E4 raw C-A P99 Z~1.007m yet matched-Z differs only−18.1mm: raw angular tails and global matched translation are not interchangeable.

Public aggregate file records every fit's seed-relative Z/XY/rotation, PCL completeness, source count, valid pairs/inliers, original-rule point-plane support/residual quantiles and timing. Full absolute XYZ/RPY/matrices and detailed support remain private. Similar fitness is not accuracy; these data do not prove or exclude different local minima/wrong associations. No NDT covariance is used as an IESKF observation.

## Fixed XY/yaw diagnostic limitation

An isolated adapter queried the SAME unchanged CUDA score at parameterized poses. Archive score at the15 full-match endpoints reproduced. The first diagnostic incorrectly minimized score rather than negative score; all those results are excluded as ADAPTER_SIGN_ERROR. During checking, exported angular derivatives and actual score differences disagreed at the seeds, so the corrected separate SLSQP diagnostic used actual score central differences1e-5, not those angular derivatives. No derivative/backend repair or old Hessian research was undertaken.

These are NOT production GLL optimizer paths. Nine solver-success messages among the corrected initial15 did not meet explicit finite-difference stationarity certification; none qualified. After correcting B's definition, only A/C10 results retain input validity; all10 remain unqualified. The new5 B diagnostics were not rerun merely to chase convergence. SciPy trial bound-clipping warnings are optimizer behavior, not velocity clipping or filter submission. Final failed iterates (not best intermediate values) and errors are retained.

Therefore there is NO QUALIFIED constrained-control basis for numerically partitioning full6D XY/yaw–Z coupling. Primary A/B/C full6D source sensitivity is valid under frozen controls, but a direct-only-vs-coupled Z conclusion remains NOT_VALIDATED. Do not use the failed diagnostic's lower score/final height as a valid result or claim mathematical infeasibility.
