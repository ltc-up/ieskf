# AC1 — three states, one immutable prior

A is the original AA first prediction. B is AA's actual partial accepted correction. C is the previously saved AB fixed-AA-final-factor local refinement of B. No state or prior is reinitialized for this comparison; C is a diagnostic state, not an actual historical update.

| State | Selected support | Own selected-factor Huber MAP | Common AA-final-factor Huber MAP | GN white norm | Dual gradient | delta Z, m | XY correction norm, m | wrapped map-yaw change, rad |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A | 599 | 1478.367250 | 6037.984206 | 0.574146 | 36.4888 | 0.000000 | 0.000000 | 0.000000 |
| B | 917 | 879.872560 | 879.872560 | 0.443617 | 30.939 | 0.936400 | 0.939525 | 0.026722 |
| C | 931 | 282.545699 | 186.036117 | 0.22479 | 11.432 | 1.182358 | 0.639320 | 0.033102 |

The common-factor column compares exactly the same factors and frozen prior. The own-selected column uses EACH state's re-queried factors, including refreshed Target multiplicities. It is a defined piecewise scalar but has changing support and weights; it is not a calibrated full-cloud likelihood or an accuracy score. C's common-factor value 186.036 does not mean its reselected objective is stationary: its reselected GN norm remains 0.224790, far above the unchanged 1e-4 tolerance.

All three satisfy the original cumulative engineering balls. B's normalized safety margins are [0.560111945754096, 0.9673549331337904, 0.9998605492878733, 0.9927089675933879]; C's are [0.5483248609043316, 0.9456895932606105, 0.9998048852398932, 0.9939621302826649], in position/orientation/gyro-Bias/accel-Bias order. Margins are `1 - norm(delta_block)^2/limit²`; positive means inside, not a physical probability. Full15 corrections, exact prior covariance and Source/Target pairs/normals are private and hash-bound. Translation is at the IMU-state origin in the world frame; attitude errors are right perturbations. The yaw column is the wrapped difference of map-frame attitude yaw, not simply the third rotation-vector component.

The frozen P and full15 prior are retained, including velocity/Bias correlations; no pose-only solution is pasted into the state. Native Source reference time matches its frozen prediction. No deskew or point motion compensation is repeated.


Publication boundary: independent research code and de-identified numerical aggregates only. Full state/P, map/Source points, event timestamps, Target identities, equipment/extrinsic information and original project source are not distributed. No independent pose accuracy, calibrated covariance or production qualification is inferred from local numerical results. See [manifest.json](manifest.json) and [REPRODUCE.md](REPRODUCE.md).
