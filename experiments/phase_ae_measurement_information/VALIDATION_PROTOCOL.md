# Spatial cross-fit identity, frozen before validation outcomes

Historic G1/G2 and all full-data density fits are contaminated for any retrospective holdout. Their scores are posthoc consistency ONLY.

NEW cross-fit training starts from the original nominal seed, original full15 Gaussian prior/P, no G1/G2 initialization. Source assignment and20m/3.5m-core map partition use the original prediction only. Train and validation Target normal fits are performed independently within their respective maps. The >=7m center gap exceeds twice2.5m normal radius. Verify actual geometry gap and normal-neighborhood identity disjointness. Validation nearest-valid residuals are reported WITHOUT the1m cutoff, together with distances/support and every predeclared block. No absolute truth is available.

For cross-fit sampling only, repeat the SAME A/B/C/D rule on TRAINING records and candidate pool of the13 original seeds applied to TRAINING Source. Do not reuse high-density-cell choices derived from all-data G1/G2. This training-only construction prevents an all-data covariance/geometry leakage in the sampling mask. Validation never influences weights, initializers or density choices.

Four new training-only fits A/B/C/D from nominal are predeclared. Compare each on identical excluded validation Source/planes. Failure of original support thresholds is preserved, never fixed by picking another fold. Full-data real fits and roots can also be evaluated there but are never called independent. Disjoint sampling/neighborhoods still need not imply statistical independence of a global ground plane/map error.
