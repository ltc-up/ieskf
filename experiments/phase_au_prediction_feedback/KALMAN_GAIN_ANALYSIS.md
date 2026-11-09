# AU2 — Why a decimetre innovation produces a centimetre correction

**VERIFIED matrix mechanism under frozen uncalibrated P/R; not covariance calibration.** All1468 updates use XYZ plus prior-anchored SO(3), with full15D gain. H includes the IMU→rear-axle attitude lever term. Independent NumPy solve/Joseph/right-Jacobian calculation matches logged K to4.45e−16, delta to6.67e−16 and postreset P to4.45e−16. Posterior symmetry discrepancy0; minimum eigenvalue5.58e−7. Finite-difference reset checks at the three predeclared median frames differ by≤7.62e−12. PSD is a numerical property, not consistency.

At each frame, lateral n is the prior body's full3D y axis in map. Let J=H_position, L=nᵀJ, and e=[n,0]. Unit-lateral-position-innovation gain is `eᵀ H K e`, retaining all state/pose correlations and lever coupling. Heading gain uses the actual yaw differential and local rotation vector corresponding to map-z rotation. Neither is read directly from Pzz or a2×2 gain. The realized correction/innovation ratio is separately reported, with a numerical near-zero denominator exclusion; it is not a direction-only Kalman gain.

| Section | Unit lateral gain mean | sqrt(mean projected prior variance) m | sqrt(mean projected R variance) m | Mean lateral innovation m | Exact rear-axle lateral update m |
|---|---:|---:|---:|---:|---:|
|1 control|.0871|.0731|.2376|−.0735|−.00131|
|3 right|.1032|.0626|.1858|−.1520|−.01546|
|9 right|.1055|.0693|.2029|−.1193|−.00955|

The current assumed prior is substantially tighter than the engineering observation noise. This mathematically yields about10% unit-position gain. The scalar projected P/(P+R) is .10330/.10567 in3/9, close but not identical to the full operator .10321/.10551. Additional mixing among observed pose channels is therefore small for this unit impulse, **conditional on the full HPHᵀ already used in both calculations**. This does not show that all state cross-covariances or lever terms are negligible. This does **not** authorize deleting correlations or establish that R is too large or P too small.

Mixed real residuals still matter. Mean IMU-position correction from position residual / attitude residual / exact rotating-lever term (m): section3 −.015382 /−.000672 /+.000589; section9 −.009287 /−.000538 /+.000279. These three sum to the actual base correction. The sign and contribution of longitudinal/vertical residuals are retained in the full position-residual contribution; the realized gain ratio need not equal the unit-lateral gain. Track-normal GNSS differences use a different frozen reference axis and give AT's−.01587/−.01220m means; there is no contradiction.

The analysis also whitens HPHᵀ by the full R Cholesky and records generalized eigenvalues and λ/(1+λ) modal responses, without mixing raw metre/radian eigenvalues. This is a local conditional information comparison, not evidence of true error distribution. P/R were never varied in the factual run. Separate fixed-state PSD counterfactuals are reported in the later stage.

An exact variance partition (covariance_partition.json) preserves signed cross terms: section3 position/attitude-lever/position-attitude-cross contributions .0039424/+.00003194/−.00005518m², totaling .0039192m²; section9 .0048148/+.00003694/−.00005589m², totaling .0047958m². The smaller lever/cross contributions here do not authorize deleting them; prior position covariance itself already reflects full velocity/attitude/Bias propagation. Longitudinal/lateral/heading covariance evolution plots use the correct post-injection attitude and reset chart for posterior projections.
