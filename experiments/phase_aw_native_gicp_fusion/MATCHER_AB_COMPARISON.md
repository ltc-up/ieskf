# 1468 same-Source/same-float-seed actual matcher transactions

Every frame uses frozen AR Deskew Source SHA and the same historical float seed; neither backend supplies feedback in this **diagnosticAB**. Both matchers actually run. CUDA results reproduce original1468 final transforms exactly (max0). small_gicp uses its actual CPU solver, not stored NDT poses. Closed-loop evidence is separately reported.

NDT1468/1468 valid+converged; GICP1399/1468 (95.30%). These are optimizer/format qualifications, not correct-peak labels. Native zero-based iteration index is preserved. Both target paths bind the same map0105 source Target; local representations/neighborhoods differ as documented.

|Common eligible raw matches|NDT|GICP|
|---|---:|---:|
|XY RMSE m|0.608|0.610|
|abs_longitudinal RMSE m|0.578|0.577|
|abs_lateral RMSE m|0.189|0.198|


Raw-reference comparison above uses **1141 common eligible historical-reference samples**. NDT own-valid reference count1197 is also retained (`own_valid_XY`); restricting NDT to GICP-success frames must not be mistaken for its full output coverage. Whole-lap raw same-input XY is nearly equal under75ms. Matching and filtered-posterior errors are separate metrics, not interchangeable.

GICP raw right3/9 lateralP95≈0.247/0.402m vs NDT≈0.290/0.438m on the common mask; segment1 improves, segment5 worsens≈0.429→0.862m. Relative final position/body direction/Yaw distributions and six preselected repeat/probe cases are in matcher_ab_metrics.json. No raw-cost or rawH magnitude is compared across objectives.

Fixed midpoint probes cover0/1/3/5/8/9, ±1.5m lateral/±.1deg yaw. GICP local solutions differ up to0.319m in5 and0.152m in3; other tested cases are closer. Same-initialization repeats agree numerically. These are local-basin/association sensitivity observations, not independent correct/wrong labels. Zero score or converged flag with zero pairs is invalid; the initial adapter failure demonstrates this distinction.

Final-factor cost perturbations: maximum absolute diagonal curvature discrepancy from GN across the36 tested axes is≈0.00170 relative. This supports a local conditional metric near these selected solutions, not rematching/global covariance. Full factorH, optimizer-lastH, gradient, cost, inliers and timing remain private.
