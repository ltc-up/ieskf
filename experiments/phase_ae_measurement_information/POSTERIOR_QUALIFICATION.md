# FILTER_POSTERIOR_NOT_QUALIFIED

## Precise probabilistic boundary

The AD ratio Zi/Di is a bounded geometric potential with a constant null floor at unsupported locations. No normalized sensor-space likelihood, sensor observation domain, map-sampling measure, plane uncertainty or source-point correlation model has been supplied. Over unbounded point space, the positive constant tail alone cannot be a normalized measurement density. Mixture weights also depend on the transformed observed point. It is invalid to assume this is the old independent point-plane R merely because sigma/Huber numbers are unchanged.

A Gaussian prior times exp(-geometric energy) CAN define a mathematically proper conditional Gibbs/energy distribution over the local state chart (the factor is positive/bounded). Laplace curvature is an approximation to that DECLARED potential, not proof it represents actual sensor uncertainty. These statements are compatible.

## Answers to information qualification

- Duplicate Targets do not become m extra Source observations, but they change null-versus-plane prior and curvature WITHOUT new physical surface information. Uniform/nonuniform sampling and actual normal refit are empirically separated.
- Exact known-plane X/Y/yaw information should be zero from the surface. The sampled-grid objective invents strong tangential confidence and can move a true/prior-consistent state toward denser records. This rejects geometric-observability sanity for using its Hessian as sensor information.
- G1/G2 pose/curvature and13-start attraction change with density; no independent actual pose/covariance calibration is available. Point, map, normal and shared-preprocessing errors are not modeled as independent innovations.
- Multiple local modes alone do NOT prove a Gaussian approximation impossible. If the UNCALIBRATED energy were taken literally, the local Laplace log-mass ratio G2/G1 is≈-2658.111, strongly favoring G1. This is NOT a physical posterior probability or evidence G1 is accurate. Unknown modes, density dependence and unvalidated likelihood prevent certifying a global Gaussian approximation. Initializer success fractions are not posterior masses.
- The strong sigma(Z)≈0.00783m has no independent accuracy/coverage validation. It is numerically reconstructible, but arbitrary information scaling, eigenvalue floors or speed clipping would not fix its measurement semantics.

## Decision and next step

**Do not inject AD Laplace P into IESKF.** AE makes zero filter updates and zero continuous runs. The target is useful as a continuously differentiable association/optimization research potential, but is NOT qualified as a trusted localization frontend or direct probabilistic inertial observation.

Prioritize explicit observation/sampling/plane uncertainty modeling and separate geometry-front-end proposals from filter-back-end information/eligibility while that model is unresolved. Retain the broader geometry-residual research direction; stop promoting THIS raw mixture Hessian as trusted posterior. This is a modeling task, not more kernel tuning or a return to NDT covariance scaling. No new objective or covariance remedy is implemented in AE. Longer/closed-loop replay is not recommended until density behavior, independent geometric agreement and posterior meaning are justified.


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
