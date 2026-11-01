# Phase AC predeclared plan

Baseline: published AB commit 5f8ddec331537c488110a58b842c57f8a5e9a735 and unchanged AA corrected inputs. No posterior gate, Q/R/P, sensor/velocity model, Source, map, Huber, normal/search rule or engineering limit is retuned.

AC1: compare the original zero-error prediction, actual AA first accepted cumulative error, and the previously saved AB first fixed-final-factor refined error. Evaluate their OWN selected-correspondence Huber MAP and also the COMMON AA-final-factor objective. Record full prior-white optimality, actual Source/Target pairs, normals, correction and guard margins. Do not equate objectives with different factor sets.

AC2: a single alternating diagnostic: freeze current correspondences, optimize the original full15 exact Huber objective within cumulative safety balls and original per-step translation/rotation trust limits, then requery. Prior center and P remain fixed for the entire frame. Local reference uses existing SLSQP interfaces (200 inner iterations, ftol=1e-12), not a new KKT derivation. Cap at 60 outer steps. Original line-search scales are retained for feasible fixed-objective backtracking; no forced acceptance or changed noise.

Interior qualification reuses AB untruncated white-step/dual-gradient tolerances and exact local pair witness unchanged. An association-stable constrained boundary solution may be reported diagnostically, but is NOT a Gaussian posterior and cannot enter filtering. No label is upgraded solely because a budget expires. Record fixed before/after cost and reselected-set cost separately.

AC3: 13 starts, fixed BEFORE outcomes: nominal zero and +/- each of six marginal-pose Cholesky axes with conditional recovery of the other nine coordinates. Start radius <=0.25 prior-white units and <=25% of every original engineering ball. These change only optimizer starting coordinates; the original prediction/prior is never recentered. No success-seed selection. Same outer/inner budget for every start. Strict numerical endpoint/association identity is reported; it is not a physical accuracy cluster.

Representative frozen checks are the same rejected-large-proposal, largest-committed-velocity-change, and well-converged end-of-budget samples used in AB. No additional favorable sample is chosen. Additional budget, if needed to diagnose exhaustion, may ONLY continue the nominal run to one predeclared 120-step cap; no parameter or seed search.

Continuous admission requires a nominal first-frame interior safe association-stable certificate, consistent posterior transaction, new solver-interface tests and representative checks. Nonnominal successes cannot replace a failed nominal by choosing the best cost. Boundary-only or unfinished association outcomes block continuous filtering. If admitted, one original-window diagnostic only; no longer run or production change. Full event data stay ignored; public results are de-identified and self-contained.
