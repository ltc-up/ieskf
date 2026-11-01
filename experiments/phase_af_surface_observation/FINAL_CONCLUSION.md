# Phase AF conclusion

Surface distances can form a mathematically explicit backend distinct from AD candidate generation. All23 NEW AF tests pass. A known horizontal plane retains exact X/Y/Yaw nulls; two/three independent normals produce the declared rank5/6. Exact Target aliases and same-identity Source copies add zero information. Genuine extra independent samples average epsilon but shared plane errors bound confidence.

A complete3-parameter shared plane model, full correlated covariance, Woodbury solve, explicit latent marginalization and Joseph15D update agree numerically. One physical plane split into independent patches is a demonstrated information-counting failure: sigmaZ0.101906→0.041730m without new geometry. Correct shared parent identity prevents this.

Real frozen diagnostics expose the remaining bottleneck. Deterministic groups are only geometric hypotheses: patch-scale changes alter identities/support and conditional sigmaZ0.0562/0.1150/0.0626m. Some fits have large scatter; covariance is transferred from a SYNTHETIC law, not estimated from map uncertainty. Source/prior/plane independence is not verified. Thus positive numeric evidence cannot certify a real Gaussian observation.

The original prediction's shared-surface proposal dZ0.830m/dvZ0.604m/s fails the unchanged single-linearization trust guard. State AND P remain unchanged. No actual Z-vz improvement has been shown. AD G1 has better surface P90 than G2 here, but the data overlap candidate generation and there is no independent pose reference; neither is selected as truth.

|Qualification|Status|
|---|---|
|SYNTHETIC_SURFACE_OBSERVABILITY|PASS|
|EXACT_DUPLICATE_INFORMATION_INVARIANCE|PASS|
|SHARED_PLANE_ERROR_NUMERIC|PASS|
|FULL15_CONDITIONAL_POSTERIOR_NUMERIC|PASS|
|REAL_SURFACE_IDENTITY_AND_SEGMENTATION|CONDITIONAL|
|REAL_PLANE_COVARIANCE_CALIBRATED|NOT_VALIDATED|
|G1_G2_INDEPENDENT_DISCRIMINATION|NOT_VALIDATED|
|REAL_FROZEN_FILTER_UPDATE_QUALIFIED|NOT_VALIDATED|
|CONTINUOUS_REPLAY_ELIGIBLE|FAIL|
|STATISTICAL_CONSISTENCY / INDEPENDENT_ACCURACY|NOT_VALIDATED|
|REALTIME / PRODUCTION_CANDIDATE|NOT_VALIDATED|

Answers to the research questions: (1) infinite-plane nulls preserved; (2) exact aliases do not enhance information when identity is retained; (3) shared tilt/offset represents known synthetic common errors; (4) independent segmentation can duplicate information, and real physical identity remains unresolved; (5) unlike AD, surface H has no sampling-kernel tangent forces; (6) no credible unique G1/G2 winner; (7) the15D interface is numerically valid under explicit assumptions, but this real transaction rejects; (8) verified plane groups/anchor frames, calibrated shared map and Source errors, and an independent time/frame/datum-qualified reference are needed before recursive validation.

Continue with frontend/backend separation rather than AD Hessian-to-P. Next priority is identifying and calibrating physical surface error/segmentation correlation from independent map reconstruction or reference data, followed by admissible frozen first-frame capture. Do not tune Q/R, suppress cross-P, loosen guards or run a continuous filter merely because this conditional posterior is SPD. No AF continuous replay or production changes.
