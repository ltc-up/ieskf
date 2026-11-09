# Numerical and actual replay validation

Final **195/195 tests pass,0 failures/0 skips** with AW_GICP_BINARY exported;183 baseline plus12 new. New coverage includes Native full-covariance chart congruence, GICP `[rotation,translation]` order/units, Schur versus diagonal conditional information, shape invariance to scalar objective units, singular information rejection, right-twist residual finite differences, common-anchor full coupled transport, bounded-journal exact round trip, and three actual C++ service tests (known3D transform, genuineROI acceptance/empty-correspondence rejection, final fullH/gradient finite differences).

No original test or threshold is removed. A preliminary ResourceWarning is preserved and fixed by context-managed close. First template compilation and lifetime/zero-pair adapter failure are separately retained; passing final tests does not erase those runs.

Fresh legacy40 actual CUDA/Deskew gives exactly equal seeds, full priorP, CUDA final pose, fullK and15D delta to originalAR. Full1468 same-input NDT re-match is also exactly equal in final poses. Real GICP repeated seeds/profiling show numerical repeatability; complete timed1468 loop differs by≤1.14e−13 in pose,≤8.89e−16P and≤1.44e−14 in delta, with identical acceptance. Parallel floating reduction portability is not promised bitwise.

GICP4134 saved P symmetry error0, minimum eigenvalue≈5.00e−7. Full delta=K×innovation is checked in actual transactions, including position/velocity/attitude/Bias coupling. OriginalSO3/Joseph/reset/delay equations are reused; increased history, artificial gain, cross-term deletion and Bias/Z clipping are absent.

Native comparison uses a global scale proxy because retainedκ is evaluated at rounded float pose while nativeC uses internal parameters. Trace-normalized shape/coupling remain meaningful; exact rawGN magnitude/floor occupancy is not reconstructed. GICP final factors are refreshed;36 frozen-factor axis-curvature trials agree locally withGN up to≈0.17% relative, not a rematching/global covariance proof.

Numerical correctness/PSD/function execution does not imply statistical consistency, calibrated measurement covariance, correct map peak, independent GNSS truth or production safety. AW3's absence of justified adaptiveR is an explicit outcome.
