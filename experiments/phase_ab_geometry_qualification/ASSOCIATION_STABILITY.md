# AB2 — fixed factors versus dynamic association

Representatives were selected deterministically: first committed frame; rejected large-proposal frame; largest actual vertical-speed correction; smallest final GN step; smallest final pair overlap, deduplicated. The last role duplicates the first, leaving four independent fixed-factor refinements. No sample was selected by improvement. Five separate implementation checks also include the borderline old convergence label.

Reused the existing exact-Huber residual/Jacobian interface and SciPy local reference from the saved final state, holding FINAL correspondences/normals/weights fixed. This is additional fixed-input diagnostic work, not an increase of the filter's four-iteration budget or a new constrained solver. For the rejected frame the diagnostic starts at the LAST ATTEMPTED internal iterate; its actually carried state remained the prediction.

| ID | Selection | Initial GN norm | Fixed-refined GN norm | Reassociated GN norm | Pair Jaccard after refinement | Guard violations at fixed solution |
|---|---|---:|---:|---:|---:|---|
| R0 | first_committed_update | 0.443617 | 7.74862e-08 | 0.22479 | 0.05 | none |
| R1 | rejected_large_proposal | 4.09879 | 4.29248e-08 | 1.67758 | 0.265533 | gyro_bias |
| R2 | largest_committed_vertical_speed_change | 1.28362 | 5.27625e-07 | 0.294363 | 0.514151 | gyro_bias |
| R3 | smallest_final_GN_step | 2.4529e-08 | 2.42598e-11 | 2.42598e-11 | 1 | none |

All four fixed-factor problems satisfy the candidate local numerical tolerances after refinement. First-frame fixed cost decreases 879.872560 → 186.036117, but requerying at that solution changes almost all pairs and leaves GN norm 0.224790. Thus local fixed-factor solvability does not solve its dynamic correspondence problem. R1 and R2 also require gyro-Bias changes outside the unchanged engineering guard; their unconstrained diagnostic minima are NOT feasible committed filter updates.

Fixed-reference function evaluations: 8, 8, 9, 2; elapsed diagnostic wall times: 97.298, 90.924, 87.131, 44.608 ms. These wall times include local reference and neighborhood probes, not the cost of changing an online iteration limit.

At each original representative pose, perturb six pose-white axes by the EXISTING 1e-4 tolerance, both signs, conditionally recovering the other nine coordinates. There are 12 probes per pose. Pair changes occurred in 2/12 probes at R0 and zero at R1/R2/R3. Different factor sets/objectives/gradients are recorded separately. We do not claim these probes found every local basin, or that unchanged pairs prove stability over a full neighborhood. The fixed-refinement solutions and nearby perturbation outcomes are different experiments and must not be conflated.

Across all 226, 48 last/final pair sets are identical; median retained pairs 0.996981, median pair Jaccard 0.994123. Most p90 normal-angle changes are at floating-point arccos resolution (about 1e-6 degrees), with a maximum frame p90 change 8.344822 degrees. Many target switches therefore preserve similar plane normals. Exact-pair qualification is conservative, not a proof that every changed pair is unreliable. First-frame initial-to-final pair Jaccard 0.058659 is a separate, large accumulated change.


Public scope: de-identified research aggregates; no sensor timestamps, global trajectories, map points, equipment/extrinsic details or restricted production code are published. All state/P/factor arrays and event-level identities remain private. Numerical qualification is not independent accuracy or statistical calibration. Code/data bindings and reproduction prerequisites are in [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
