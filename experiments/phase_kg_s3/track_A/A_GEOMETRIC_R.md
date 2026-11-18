# A1 conditional directional R

Implemented `sandwich_information` and `DirectionalObserver` as an uncalibrated engineering comparison. This is not the legacy Fitness R and not a calibrated inverse Score Hessian.

At the final original Align pose, the independent service evaluates the original Score/H and partitions identical current source points into fixed 8 m cubes in Source coordinates. All multiple-neighbor contributions from a point stay inside its block. It evaluates each block with the same original Score; all blocks sum to the complete Score/g, tested exactly on the synthetic GPU sample. This initial implementation has many GPU callbacks and is expected to be expensive; complete cost must be retained. Fewer than seven blocks explicitly rejects the sandwich update.

In normalized translation/RxRyRz units, retain positive identifiable curvature directions only. Build the centered cluster-gradient meat `M=K/(K−1)Σ(g_b−ḡ)(g_b−ḡ)ᵀ`, then `R=C⁻¹ M C⁻ᵀ` in that subspace. Dimensionless variance has an explicit floor of one, corresponding to conditional scales 0.5 m and 3 degrees. Null curvature contributes zero observation information. Numerical rank cutoffs never introduce information in absent directions. Map all 6×6 cross terms into the existing right-error rotation innovation chart before one complete-P ExternalPose update.

These assumptions do not remove map mismatch, biased estimating equations, association discontinuities, or temporal dependence. No independent calibration or held-out consistency result has yet been obtained. The original H has the reported pitch derivative defect; the user-approved correction applies only to A2. A1 therefore preserves and labels this original-curvature limitation rather than extending the correction without permission.

## Completed continuous evidence

On 0105, 7217/7229 accepted frames have all directions clamped to the dimensionless variance floor; 99.9677% of individual directions are clamped. On 1013, every direction of all 7770 accepted updates is clamped (largest raw normalized variance 0.522024). Thus the observed R is almost entirely the pre-frozen 0.5 m/3 degree engineering floor plus chart transport, rather than measured sandwich anisotropy. This negative mechanism result is retained without floor retuning. Held-out statistical calibration remains absent.

0105 original freeze lacked full chart/basis matrices. Exact floor-case reconstruction recovers 7194 accepted full-rank frames; 35 accepted matrices remain unavailable. The 1013 passive-log-only freeze records full chart R and projection for every accepted frame. This logging version difference is explicit in `A_CONTINUOUS_RESULTS.md`.

A2 completion does not improve A1 statistical qualification: A1 remains a separate one-Pose-update conditional R experiment, while A2 optimizes the original Score plus full Prior and constructs a separately qualified conditional P. Neither complete-run SPD nor A2 large-correction evidence calibrates A1 sandwich floors.
