# Negative results and blockers

Qualified real NDT/GNSS global fusion/recovery: FAIL. No calibrated NDT R,no qualified map-base GNSS contract,no independently referenced true peak. Strict controlled A/B/C qualification fails because historical runtime NDT enable,map version and acceptance decisions are missing. C global availability0% is not a successful locator. Production deployment NO-GO.

Local vertical behavior remains unacceptable for a claimed stable3D solution: actual wheel dZ reaches4.177m; sigmaZ51.783m is model-conditional,not calibrated true error. No Q/R search,height fixing,cross-P deletion or clipping was attempted. Map separation does not solve this failure.

Initial attempt to source installed GNSS package at its prefix failed; corrected to its share/package local_setup script and raw ROS deserialization succeeded. No raw GNSS message was replaced by converter pose or an INS truth surrogate. Local fix-code schema/SDK only defines0–3 while observed values5/7/8 and production code checks8; this is a protocol qualification blocker,not a claim of bad fixes.

Implementation review moved invalid-bound checks before coordinate commit,retained independent dimension expiry,added same-time hypothesis transport,stale-hypothesis pruning skip,source contract/clock checks and separate convergence health fields before final tests. Synthetic atomicity and transport tests pass. No real source was allowed to pass these unverified prerequisites. Complete real-loop improvement,statistics,wrong-peak rate,per-LiDAR operation and20Hz remain UNQUALIFIED.
