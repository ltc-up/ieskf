# Conditional posterior binding, not continuous fusion

No covariance is updated within alternating local/outer iterations. Every iteration uses the SAME original prediction/P; the same cloud is not counted as multiple observations.

For the single nonnominal INTERIOR endpoint only, reuse the original unconstrained final-linearization/IRLS posterior formula and error reset once, then use the UNCHANGED AB atomic selector. State injection reproduces all15 solved coordinates, including correlated velocity and Bias; no component is removed or clipped. The candidate covariance is finite SPD (minimum eigenvalue 1.87384e-08) and the matching state/P transaction succeeds numerically.

Model-conditional sigma Z becomes 0.009382519m from 1.077426997m. This is even tighter than AA's partial-update value; convergence does NOT automatically calibrate point/normal/map/preprocessing uncertainty. The output remains `UNCONSTRAINED_LOCAL_IRLS_APPROXIMATION_UNCALIBRATED`, not a statistically verified posterior.

The diagnostic transaction is DISCARDED: actual filter updates and continuous runs are ZERO. It is not a new carried velocity sample or proof that vertical-speed spikes improve. For the nominal periodic candidate the unchanged eligibility fails, so both prior state and P are retained exactly; no partial candidate covariance is committed. Boundary gyro-Bias solutions have no computed Gaussian posterior.

This binding check reuses existing numerical posterior machinery instead of repeating AB's proof suite. It establishes implementation semantics at one isolated certified endpoint; it does not establish robust capture, information credibility or a continuous filter.


Publication boundary: independent research code and de-identified numerical aggregates only. Full state/P, map/Source points, event timestamps, Target identities, equipment/extrinsic information and original project source are not distributed. No independent pose accuracy, calibrated covariance or production qualification is inferred from local numerical results. See [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
