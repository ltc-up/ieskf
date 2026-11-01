# First-frame conditional nonlinear solve

Executed: original first prediction, full15 P, native Source/Target, AF saved groups/Sigma and all original engineering bounds. AF one-linearization dZ0.830381m/dvZ0.603583m/s is reused as A, not recomputed as a new baseline. B starts ONLY from the same original prediction at each of5/2.5/10m; no G1/G2 or selected seed initializes it. Five NEW interface tests pass; AF23 tests and older algebra/capture experiments are not rerun.

For a fixed Source→surface hypothesis assignment a, build the entire `R_a=D+Jplane(x_pred) Sigma Jplane(x_pred).T` at ORIGINAL predicted coordinates. Hold this matrix constant throughout the local nonlinear solve:

`E_a(z)=0.5||L^-1 z||²+0.5 r_a(x_pred boxplus z).T R_a^-1 r_a(x_pred boxplus z)`.

This is a declared frozen-R conditional surrogate. It is NOT the full state-dependent plane-uncertainty likelihood; if R(x) varied, covariance derivatives and logdet terms would be required. New association produces another labeled E_b, not a globally smooth/common merit. Within each solve the gradient uses the same frozen R and full original prior. No AD cost/Hessian, hard-MAP covariance trick or independent reapplication of the same observation.

A feasible trust-SQP step minimizes the GN quadratic in full15 prior-white coordinates, subject to original cumulative position/attitude/gyro-bias/accel-bias norm limits and per-step translation0.5m/rotation0.05rad. SLSQP solves only this constrained quadratic; actual nonlinear decrease is separately Armijo checked. Frozen prior stays unchanged across60 allowed inner steps and20 outer association queries. These larger budgets are offline diagnostics, not production parameter changes. A final candidate needs original KKT stationarity/complementarity tests; an interior solution also needs existing1e-4 untruncated whitened GN step and decrement. Small scaled steps alone cannot qualify. No state or covariance is submitted.

|Patch m|Status|Outer|Accepted inner steps|dZ m|proposed dvZ m/s|Final Source support|Fresh-association GN white|CPU s|
|---|---|---:|---:|---:|---:|---:|---:|---:|
|5|CONDITIONAL_LOCAL_AND_ASSOCIATION_STABLE|9|27|1.035756|0.729555|2160|3.00331e-08|1.562|
|2.5|LOCAL_NO_DESCENT_DIRECTION|5|19|1.033150|0.728073|1021|0.379043|0.658|
|10|OUTER_BUDGET_EXHAUSTED|20|64|0.904287|0.630586|2856|0.542599|3.642|

**5m conditional endpoint:** cumulative position norm1.147673m<2m; attitude0.040528rad<0.15; gyro-bias0.00046178rad/s<0.03; accel-bias0.0202107m/s²<0.3. All final constraint margins positive. Maximum accepted translation step0.500000000018m (1.8e-11m floating solver roundoff; no limit relaxed) and rotation step0.023843rad. Final source-plane mapping exactly unchanged, support2160/4005 and1578 unique Target records. Stationarity inf8.67e-5, GN white3.00e-8 and decrement1.25e-6 satisfy inherited tests. Twelve supplementary endpoint microprobes were declared after endpoint observation but before probes: all +/- six pose-prior axes at1e-4 white radius retain exactly the same factors. Frozen-R directional-derivative differences max4.77e-7 /3.35e-6 at white steps1e-5 /1e-6. This certifies only a tiny local witness, not a broad basin or physical truth.

**2.5m failure:** local subproblem reaches the2m cumulative position boundary. Its final local stationarity0.001752 exceeds0.001 tolerance; a subsequent quadratic direction is not a valid descent, so NO_DESCENT_DIRECTION is retained. Reassociation would further change34% of factor union; final fresh-objective stationarity572.25, GN white0.379. This is not proof that no feasible optimum exists. No tolerance loosening or solver retry was used to rescue it.

**10m failure:** all20 fixed-assignment solves have local interior evidence, but reassociation continues to alter the local problem (last Jaccard0.99093). Fresh final GN white0.5426/decrement1.3148 fails, so OUTER_BUDGET_EXHAUSTED remains. No exact cycle or impossibility claim; more budget might behave differently but was not tried.

Mechanism: AF's single step exceeding0.5m is not a proof that the FINAL legal region is empty. At baseline5m, a legal multi-step path exists and final dZ is even larger1.036m. This does not validate the pose or shrink velocity uncertainty: full15 cross-state prediction still proposes dvZ0.730m/s, and all transactions are discarded. Strong dependence on uncalibrated surface groups remains. Conditional costs change supports/R across outer queries and are never ranked as a single common likelihood. Runtime includes this CPU frozen solve/query path, not production scheduling or20Hz feasibility.
