# AV numerical validation

195 tests pass,0 fail;12 new tests relative to AU183. New directed cases verify:
- same static linear-Gaussian packet information for one R_packet update versus five repeated5R_packet updates, including full covariance; a single5R_packet update demonstrably loses information;
- original legacy covariance default remains exact;
- consistent guarded update matches original full15D state/P and stays SPD;
- unchanged full-pose Jacobian agrees with finite differences with a nonzero lever;
- a converged/low-fitness large innovation is rejected with prior mean/P intact;
- competing local basins, weak/partial geometry, poor map support and failed probes have separate eligibility outcomes;
- consensus does not certify a globally correct peak;
- delayed correlation-group deduplication prevents repeated fusion;
- invalid observation noise fails closed.

Existing SO3, right-error F/G, full Joseph/reset, delay-history, numerical PSD and AU independent reconstruction tests remain unchanged. Native-pose R uses metre² translation and local-right rotation radian²; angular mapping is isotropic here and not an untransported Native Hessian. No eigenvalue-to-R mapping or state cross-term deletion occurs. Geometry eigenvalues are eligibility diagnostics only.

Actual original-policy40 raw CUDA/Deskew replay has0 maximum seed/final-pose/15D-delta discrepancy against AR. Guarded40 raw CUDA/Deskew completes40/40 accepted with local three-start agreement. Numerical validity and short-run convergence do not establish statistical consistency, physical correct peaks, full-lap improvement or production qualification. True normal-lap and held-out behavior are reported after execution.
