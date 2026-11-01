# AB4 — one eligibility mechanism

Only the commit decision is changed. The historical solver, four-iteration budget, robust loss, nearest-neighbor/normal rules, prediction/wheel model, initial state/P, noise and engineering guards remain identical.

For the final actual candidate:
1. Recompute the exact objective gradient and untruncated GN direction with the same frozen prior.
2. Require BOTH prior-white GN norm and GN-metric dual-gradient norm <= the already frozen 1e-4 numerical tolerance.
3. Require exact Source–Target pair equality between the last linearization and final candidate, and between the final candidate and its untruncated next GN proposal. Normals are tied to the same immutable Target cache. Retain original support and engineering guards.
4. Require finite valid candidate state and symmetric positive-definite P. Commit candidate state and matching conditional IRLS P atomically; otherwise retain both frozen prediction objects. Do not clip state, reset Bias, delete correlations or retain a posterior P from an uncommitted proposal.

This uses no fitted turnover fraction, tuned velocity bound or changed noise. It is NOT parameter-free: 1e-4 is an existing numerical tolerance, newly applied to a dual-gradient criterion as a conservative policy. Exact pair equality can be stronger than necessary on a plane and is only a local witness, not full-neighborhood stability. Neither this gate nor its data-dependent rejection is statistically calibrated.

The covariance formula is unchanged and remains a conditional fixed-linearization/IRLS approximation, used only after the candidate numerical qualification. No hard-constrained MAP posterior is injected. This makes update semantics explicit; it does not solve map uncertainty or point correlation.

Eight new focused tests PASS: swapped Source–Target pairs despite equal Target sets; tiny applied step despite nonstationary next step; dual-gradient requirement; atomic rollback; nonfinite candidate rollback; matching covariance on commit; finite-difference check of the new white-gradient interface; nonfinite metric rejection. Existing U/V/X/Y campaigns were not rerun. Five frozen implementation checks match the table-based rule and verify exact rollback or retained historical state/P.

The candidate is implemented in `src/geometry_ieskf/qualification.py`. A rejected frame retains the proposed delta only under an explicitly proposed label; committed delta is zero. A candidate can be safe as a transaction yet fail as a localization filter, as this phase demonstrates.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
