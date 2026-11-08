# Current system diagnosis

Evidence below is inherited, not rerun. See [AA](../phase_aa_z_validation/FINAL_CONCLUSION.md), [AE](../phase_ae_measurement_information/FINAL_CONCLUSION.md), [AF](../phase_af_surface_observation/FINAL_CONCLUSION.md), [AH](../phase_ah_deskew_z_ablation/FINAL_CONCLUSION.md), [AI](../phase_ai_ndt_seeded_geometry/FINAL_CONCLUSION.md), [AJ](../phase_aj_ndt_pose_update/FINAL_CONCLUSION.md).

| Mechanism | Numerical evidence | Classification / architectural implication |
|---|---|---|
| Nonstationary geometry accepted | AA 226/233 acceptances at iteration limit; first sigmaZ 1.077427→.015979m | Solver/commit contract; require candidate vs accepted transaction separation |
| Full covariance transmits information | AA subsequent corr(Z,vz)=.985528; conditional coefficient19.417368/s | State coupling, mathematically expected; physical correctness needs credible measurement |
| Wrong information semantics | AE known plane falsely shrinks tangential sigma1→.0745m, yaw5→.6505deg; density drift5.44cm | Sampling-energy Hessian is not physical information; no empirical scale can fix missing model |
| Shared errors count multiple times | AF independent patch split sigmaZ .101906→.041730m without new geometry | Observation identity/correlation; carry map/source parents and shared errors |
| Frontend changes insufficient | AH strict0/15; AI original-budget0/5 each start, E1 still fails extended test | Do not change Deskew or increase iterations as architecture remedy |
| Strict qualification loses source | AB strict continuous0/284; AJ66/284, gap5.50s | Reliability manager must expose lack of accepted constraint, not claim sensor absence |
| Planar observation affects vertical state | AJ LiDAR max actual dZ .310943m/dvz .160987m/s; L079 induced dZ−.551m rejection | Full-P effect; map/local boundary can isolate feedback |
| Wheel also affects vertical state | AJ actual wheel dZ4.13669m/dvz .559931m/s | Local motion backend remains a separate unresolved problem even after map isolation |
| Missing height information | AJ final sigmaZ47.596m; no independent truth | Source observability deficit; layering cannot create Z information |

Map holes/repetition are source limitations; true wrong-peak frequency and real NDT covariance remain UNVERIFIED. Low fitness, convergence and SPD P are not correctness evidence. No current evidence proves a particular historical match was a wrong peak. Neither AE energy Hessian nor raw NDT Hessian is accepted as calibrated pose covariance.

Recommended structural changes address feedback damage, provenance and availability semantics. They do not solve missing height, wheel slip, bias observability or true-peak identification. Keep physically derived local p/v/attitude/bias cross-correlation; reject incorrect measurement models rather than erase P. A single optimizer trust step is not a Kalman innovation model or posterior certificate.
