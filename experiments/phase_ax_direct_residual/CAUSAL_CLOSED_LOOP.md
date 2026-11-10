# Actual raw-data closed-loop execution

All A/B runs use loop44 oldeav24 wheel, Bosch IMU, actual three-LiDAR raw records, original extrinsics/crop, fixed map0105, actual CUDA Deskew and original bounded500ms historical replay. GNSS and historic matcher final poses never enter the update. Only the common lawful initial production state is reused; subsequent Source/seed/state/P belong to each candidate.

A uses GICPFactor current-state correspondence/metric and normal-subspace residual. Each iteration queries the actual residual service, not a final RegistrationResult.H or stored pose. B uses actual CUDA-built native Gaussian relationships and reconstructed raw surface directions. Both observers are called at the original native Header inside delayed replay and affect future own Deskew/prediction. Filter state stays15D; temporary6D nuisance is marginalized once per scan.

|Run|Requested/processed|Accepted direct updates|Outcome|
|---|---:|---:|---|
|A40 refresh-every-step|40/38|0|retained convergence/loss failure|
|A40 patch-only fixed-inner|40/40|4|retained validity-domain failure|
|A40 linear-common|40/38|0|retained nonlinear model/solver failure|
|A40 nonlinear-common|40/40|40|function verified|
|A single lap|1468/1468|1468|73.350s, function verified; accuracy failed|
|A continuous three laps|4134/4134|4134|206.650s, no per-lap reset; accuracy regresses|
|A diagnostic repeat|1468/1468|1468|exact seed/P/delta parity|
|B kernel-shape attempt|40/38|0|no qualified surface factors|
|B raw-moment surface40|40/40|40|short function verified|
|B requested lap|1468/531|451|26.500s, severe Z/state loss; FAILED|
|B three laps|NOT_EXECUTED|0|lap prerequisite failed|

`converged=false` in the service packet means no independent pose optimizer was run. Actual direct convergence is `update.converged`/conditional MAP qualification. Original ledger key`ndt`, component-count fields and observation-loss reason names are reused schema, not evidence that all six directions are observed or that CUDA pose matching occurred. Public executed_runs.json annotates these semantics; originals remain private.

Internal iterations refresh associations twice then converge against a frozen likelihood, not a globally rematched cost. Across A lap,175 outer refreshes increase objective relative to the prior frozen trial, while accepted frozen line-search costs never increase. This preserves AD's warning rather than hiding it behind an end flag. No correct-peak certification follows.
