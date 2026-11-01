# Frozen15D update interface

Executed after the synthetic prerequisite gate: one baseline frozen linearization at ORIGINAL prediction with prediction-selected surfaces. Candidate locations are scored separately, not fused as pose measurements. No AD Laplace information, hard-constrained MAP posterior, repeated same-frame update, prior modification or velocity clipping.

Let P=L L.T, A=H L, M=I+A.T R_eff^-1 A. Solve

`delta = -L solve(M, A.T R_eff^-1 r)`
`P_chart = L solve(M,L.T)`.

The equivalent gain is K=P_chart H.T R_eff^-1. Joseph `(I-KH)P(I-KH).T + K R_eff K.T` agrees, preserving full15 correlations. Explicit joint state/plane latent elimination also agrees. Error injection is p/v/bias additive and R_new=R exp(deltaTheta); reset Jacobian chart_map(delta) is applied once. Rejection returns exact state/P copies. No same-frame posterior is recursively reapplied.

|Conditional scenario|proposed dZ m|proposed dvZ m/s|conditional sigmaZ m|Joseph relative error|isolated transaction|
|---|---:|---:|---:|---:|---|
|Shared surface latent|0.830381|0.603583|0.056174|9.78e-14|REJECTED: original single-linearization trust|
|Incorrectly independent diagonal of SAME effective R|1.031798|0.737740|0.033789|1.88e-11|REJECTED: original cumulative guard|

Prior sigmaZ=1.077427m, sigmavZ=2.010486m/s; shared conditional sigmavZ=1.859248m/s. Full other-state recovery differs from K_np deltaPose by6.41e-12; velocity/bg/ba norms of the shared proposal are0.700998m/s,0.00035058rad/s,0.0180915m/s². These are mathematical proposals, not propagated spikes. Shared conditional covariance is SPD (minimum eigenvalue4.34e-7 in declared mixed physical coordinates, not a dimensionless quality score).

The diagonal case is an explicit WRONG correlation model, not a candidate R choice. Both retain the original numerical epsilon and same plane marginal variance. It illustrates the effect of removing shared correlations, not superiority of a chosen uncertainty multiplier.

Actual committed state/P corrections are zero: both reject atomically; every real diagnostic is discarded even if an isolated transaction were feasible. The0.056m value is not a submitted posterior, and comparing it with historical AD0.00783m does NOT validate either: objective, linearization and uncertainty hypotheses differ. Exact physical covariance is uncalibrated; group independence/prior–Source independence/outliers are unresolved. No nonlinear convergence or valid first-frame capture is established by this linear reference.

Qualification: FULL15_CONDITIONAL_POSTERIOR_NUMERIC=PASS under declared assumptions; REAL_FILTER_UPDATE=NOT_VALIDATED; CONTINUOUS_REPLAY_ELIGIBLE=FAIL.
