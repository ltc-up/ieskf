# AD5: state/P semantics and continuous decision

## Executed local energy diagnostic

The nominal polished endpoint is interior and locally stable under the new energy. A full15 Hessian is differentiated from the exact analytic gradient, including weight derivatives, in prior-white coordinates. SPD is checked at three stencils. Compute the LOCAL ENERGY Laplace approximation P_chart=L solve(H_white,Lᵀ), then P_local=G P_chart Gᵀ with the same error-injection chart derivative G. No repeated covariance updates during solving; the full15 prior remains frozen. Original cross-state effects are retained, with non-pose conditional-recovery error 3.31e-13. No state component is clipped.

At the nominal point, the Hessian minimum white eigenvalue is approximately 1.0 across stencils. The local P is finite SPD; the 1e-5 versus 1e-6 stencil relative P difference is 4.43e-9 (1e-4 versus 1e-5: 5.06e-5). The existing atomic selector gives matching state/P in an ISOLATED diagnostic (state injection discrepancy 7.22e-15); rejection retains both original objects exactly. This transaction is DISCARDED. Filter updates=0.

This establishes `NUMERIC_ENERGY_LAPLACE_TRANSACTION`, not a trusted LiDAR posterior. Model-conditional sigma(Z)≈0.00783026m from 1.077427m — even tighter than the old partial posterior. Convergence did not make geometric information calibrated. Huber, map error, shared preprocessing, point density and local-mixture mode ambiguity are not justified as independent measurement R.

## Why no continuous replay

A numerical stationary point from the original prediction now EXISTS. However the unified merit substantially changes measurement semantics. At that endpoint the plane-residual white gradient norm is **1895.359509**, association-kernel contribution **1894.956283**, prior contribution **1.424878**, total **6.13e-10**. Strong cancellation shows association/support-density forces materially determine the solution; dropping their derivatives would be mathematically wrong, but retaining them does not prove a physically correct map pose. A duplicate-Target synthetic test confirms the objective is NOT density invariant.

The old IRLS/Joseph posterior cannot simply be inherited for this new mixture. The isolated energy-Laplace matrix has a precise local curvature meaning, but its interpretation as a recursively justified sensor posterior is not validated. The two local modes and sharper sigma(Z) add reasons not to equate numerical convergence with information credibility.

Decision: **NO 14.70-second run; no longer replay qualification**. This is not a claim of initial-capture impossibility: local energy capture passed after diagnostic polishing. It is a separate failure of validated recursive posterior/measurement qualification plus an unintegrated roundoff-sensitive solver endpoint. No new velocity trajectory, accepted delta-vz or continuous LiDAR-retention result exists. Do not infer improvement from zero committed updates.

Next minimal work should justify the new plane/support-density energy and its information accounting, with a reproducible high-precision solver certificate, before carrying a posterior forward. No further association candidate, noise tuning, extra initializers or gate relaxation was tried in AD.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
