# Phase AM test plan

Parent is committed/pushed AL e9f86f1a56a4d0f93870006c19606944d7cad30f. Implement diagnostics only, retain CUDA-NDT as external service. All284 original Sources are assessed against the same full Target. Frozen settings are declared before execution:2m nearest-support distance,20-neighbor PCA,2m spatial cells,1500 deterministic samples,10m angular length scale,planarity>.2 and numeric relative rank1e-6. They are research resolutions, NOT accuracy-derived gates or noise.

Synthetic automatic checks: plane rank3,orthogonal rank5,multi-normal rank6,one-sided wall rank3,longitudinal corridor null,missing surfaces,units/duplicate invariance,missing support,complete-link cluster ambiguity,chain prevention,single-start cannot verify uniqueness.35 actual CUDA jobs are predeclared: frame indices0,71,142,213,283, seven seeds each (recorded current AJ prior, ±1.5m x/y, ±.1rad yaw). No result-based seed/window selection. Record cold and warm request-group latency rather than multiply nominal timing.

Map-hole intervention removes one original-seed-centered half-space for first predetermined frame. It tests sensitivity to support removal, not whether original map has a true hole. Preserve AMBIGUOUS without qualified independent reference. Never fuse multiple starts or convert eigenvalues/fitness to R.
