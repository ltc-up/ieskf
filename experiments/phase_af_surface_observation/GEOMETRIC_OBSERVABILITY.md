# Geometric observability

Executed:23 new AF tests PASS; no historical U/V/X/Y suite rerun. Analytic surface H, finite differences of all15 state columns and3 plane parameters agree. Six-pose singular spectra and complete synthetic statistics are in key_results.json; synthetic data can be recreated without A2RL inputs.

| Known geometry | Pose rank | Geometrically unobserved |
|---|---:|---|
| Infinite horizontal plane, spread Source |3|X,Y,Yaw|
| Two orthogonal planes |5|translation along intersection|
| Three independent normals |6|none for this spread layout|
| Several finite local planes, varied normals |6|none for this layout|
| Two finite coplanar Source footprints |3|X,Y,Yaw|

A single point or collinear spread need not constrain both roll and pitch; the above ranks rely on the declared spatial layouts. Finite patch boundaries are association support, never physical-edge residuals. Thus bounded map sampling does not create tangential information. Multiplying by positive definite R_eff cannot invent rank. No eigenvalue floor, artificial direction weight or target-density derivative is used.

AD's historical exact-plane X/Y/Yaw confidence contraction was caused by the association energy. AF surface H has exact zero columns in those directions, independently of Target copies. An independent diagonal prior retains their variances. A correlated prior can transfer observed information into other coordinates without making the missing geometric directions directly observable; these are different statements.

Qualification: SYNTHETIC_SURFACE_OBSERVABILITY=PASS. Physical completeness/correctness of real surface extraction is not established by these rank tests.
