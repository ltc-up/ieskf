# Phase AD frozen plan

Baseline is committed Phase AC b8b45d0. Fetch verified remote main equals this commit. No production, inertial/wheel, noise, prior, Source, map, normal validity or safety changes.

1. Reuse the last two original and last five nearest-valid nominal states, not new fitted states. Sample each adjacent segment at 33 predetermined fractions. Isolate every detected selector/weight change with 40 bisections; record one-sided values/gradients at offsets 1e-3 through 1e-8 of segment length. These are parameter offsets, not metres. Preserve fixed-factor and queried-factor costs separately. Finite-difference smooth interiors, never infer a gradient at a jump from a central difference.
2. Examine original accepted fixed-factor directions against the reselected objective. Preserve per-factor identity, weights and losses privately; publish only deidentified aggregates and boundary diagnostics.
3. Only after direct evidence, freeze ONE explicit association-objective candidate before optimization. Derive and test the complete derivative, including association-weight derivatives. No parameter search.
4. Reuse A/B 13-start results from AC. Run C from the SAME 13 preset optimizer seeds, never recenter the Gaussian prior. Preserve original cumulative balls, step trust limits and Huber. Report ALL starts. More computation is offline only. Candidate budget at most 60 trust stages, 200 SLSQP iterations per stage, same original scales, fixed prior-white tolerance 1e-4. A candidate objective is not a calibrated likelihood.
5. A nominal endpoint must satisfy objective-consistent optimality and original safety, with reproducible neighborhood behavior. State/P atomic and probabilistic interpretation are separate gates. No continuous replay if either capture or posterior qualification fails. No posterior from a mixture Hessian by assumption.

Public artifacts contain independent research code and nonreconstructive aggregates only. Full state, map, time, point identities and private matrices remain in ignored data. Failures and costs retained; tests do not repeat old algebra proofs.
