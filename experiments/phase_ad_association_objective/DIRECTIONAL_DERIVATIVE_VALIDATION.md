# AD2: optimization and actual queried merit

## Direct direction checks

At each stored edge, use exactly the old fixed-factor accepted direction and frozen prior. The tiny alpha=1e-6 step remains in the active cell and descends. However at the full step:

|Edge|Frozen-factor change|Queried old-merit change|
|---|---:|---:|
|original 0|-0.001145057|-0.193904790|
|original 1|-0.001138334|0.193904790|
|nearest_valid 0|-0.007586608|0.955007042|
|nearest_valid 1|-0.017232484|-0.492820488|
|nearest_valid 2|-0.005598867|-0.079283328|
|nearest_valid 3|-0.005288731|-0.456937565|
|nearest_valid 4|-0.004802845|0.074034338|

All seven frozen objectives decrease; **three of seven queried endpoint changes increase**. Therefore a correct inner descent direction can cease to be a descent step after crossing association boundaries. In particular the reverse original edge climbs +0.193904790 while its frozen factor loss decreases -0.001138334. No derivative exists at a nonzero value jump; a central quotient across it diverges and must not be called a gradient.

Within 181 unchanged-signature cells, central directional differences agree with the analytic frozen gradient (maximum absolute error 3.02e-07, scalar segment parameter). Near boundaries, separately saved LEFT and RIGHT differences use only same-signature subintervals. At the original switch analytic one-sided slopes are about -0.00114124 and +0.00114213; the 0.193896 jump is separate from either slope. Very small offsets have expected floating-point cancellation; no false central-gradient test is applied there.

## New objective derivatives

The [single candidate definition](CANDIDATE_DEFINITION.md) includes the association-weight derivative. Nine new synthetic checks pass: full15 finite differences at nonzero attitude/nonzero correlated prior; unsupported-source constant; radius value/gradient continuity; plane tie; invalid plane exclusion; duplicate-map sensitivity; permutation/repeatability; loss bounds; Huber gradient continuity. These are new objective-interface tests, not repeated U/V propagation proofs.

Across all 13 real frozen raw endpoints, six pose-white directions at epsilon=1e-6 have maximum absolute analytic/finite-difference error **1.84e-05**. Offset sweeps 1e-4/1e-5/1e-6 expose truncation/cancellation rather than selecting a favorable offset. At the same 43 old switches, new-objective two-sided differences shrink with offset: maximum 0.00196773 at 1e-3; 1.96773e-5 at 1e-5; 1.96742e-8 at 1e-8. This is consistent with the declared C1 construction, unlike the old finite jumps.

The direct mechanism is joint: hard support/validity selection, unequal-loss valid-plane selection, repeat-weight changes, and a line search evaluating a different frozen merit. The original inner derivatives are not indicted by these observations.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
