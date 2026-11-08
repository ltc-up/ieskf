# Same raw normal-input A–E
All40 raw triplets, map, initial state/P, inertial/velocity models and preprocessing parameters agree. Feedback changes each branch's later Deskew/seed/Source legitimately; these are causal branches, not identical frozen fit observations.

| Branch | Actual pose updates | Model Z range(m) | Notes |
|---|---:|---:|---|
|A IMU/wheel|0|4.364|No map, not a complete localization algorithm|
|B XY/yaw|28/40|4.137|Longest no-update interval~0.60s; Z still coupled through full P|
|C XYZ/yaw|30/40|1.237|Longest gap0.30s; Z used, engineering R only|
|D geometric6D subspace|19/40|1.491|All rank6; real partial subspace not demonstrated; longest gap0.65s|
|E Map/Odom shadow|0 local|4.373|Native-reference local priors/P exactly match A; current output stamps differ|

C's requested20s continuation evaluates55/401 frames, accepts32, then stops after0.55s without accepted observation. No full-lap comparison or independent XY/Yaw/Z/speed RMSE exists. AJ66/284 is a different historical input window, not a controlled improvement denominator. This normal-window new75% retention demonstrates implemented feedback, not a validated performance gain.

Input eligibility follows user provenance: older loop44 is used; new-series wheel channels before loop10 are not trusted. GNSS remains unqualified and is not fused. No fake3D Kistler, fixed Z or covariance cross-term deletion. See aggregate plot; it visualizes model behavior, not accuracy.
