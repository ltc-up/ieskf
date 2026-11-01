# Phase AG conclusion

**A legal conditional first-frame path exists for the AF5m model, but a trustworthy real observation is not established.** Data independence and map covariance, not merely step length, still block closed-loop qualification.

1. Existing data:9 distinct full-PCD SHA256s,8 recording metadata and reference products were inspected. None supplies a complete independent acquisition/mapping-membership/datum/extrinsic chain for plane calibration. The closest inspected earlier reference ends about61s before the first frame; later reference begins about121s after. DATA_INSUFFICIENT; no within-map bootstrap or false ground truth.
2. Physical plane identities remain geometric hypotheses. AF scale splitting/merging, unresolved components and high fit scatter are retained; unknown patch errors are not assumed independent.
3. Major unmodeled correlations include shared map trajectory/datum, cross-patch common surfaces, correlated lidar samples/common Deskew, and map/Source/prediction shared information. No real Sigma entry was calibrated.
4. AF dZ0.830m exceeds the single-step trust region but does not imply absence of a legal endpoint. From exactly the original prior, baseline5m reaches dZ1.035756m in9 outer/27 accepted inner steps; total position norm1.147673m and all final engineering constraints pass. Final source-plane mapping is unchanged; inherited optimality tests and12 small probes pass.
5. This is a frozen-R conditional surrogate, not a full state-dependent likelihood. Original15D P is frozen throughout; observation noise is frozen at original predicted coordinates per factor assignment. No same-frame independent repetition or state/P recursion.
6. Scale sensitivity is substantial:2.5m ends as an unqualified boundary partial solve;10m is still associating after20 outer queries. There is no claim of global convergence, broad capture basin, unique true pose or improvement over G1/G2.
7. No short closed-loop admission: real surface identity/covariance and independent validation fail required evidence conditions. The5m proposed dvZ0.729555m/s and conditional sigmaZ0.032654m are not submitted filter outcomes.0 state/P commits and0 continuous runs.

|Qualification|Status|
|---|---|
|EXISTING_DATA_PROVENANCE_AUDIT|PASS (bounded identity/limitations audit)|
|INDEPENDENT_MAP_CALIBRATION_DATA|FAIL / DATA_INSUFFICIENT|
|PHYSICAL_PLANE_IDENTITY|NOT_VALIDATED|
|REAL_MAP_SOURCE_COVARIANCE|NOT_VALIDATED|
|NEW_FIXED_R_OPTIMIZER_INTERFACE|PASS (5 new tests)|
|FIRST_FRAME_CONDITIONAL_CAPTURE_5M|PASS under AF hypotheses|
|CROSS_SCALE_CAPTURE_ROBUSTNESS|FAIL|
|INDEPENDENT_CANDIDATE_ACCURACY|NOT_VALIDATED|
|REAL_STATE_POSTERIOR_COMMIT_VALID|NOT_VALIDATED|
|SHORT_CLOSED_LOOP_ELIGIBLE|FAIL|
|STATISTICAL_CONSISTENCY / PRODUCTION_CANDIDATE|NOT_VALIDATED|

Next priority is obtaining verifiable physical surface identity and independent map/Source/reference error evidence, not relaxing the trust domain or tuning velocity/Q/R. Preserve the conditional multi-step solver as a diagnostic of path feasibility. Do not transfer the synthetic plane Sigma to production. If suitable calibration/reference cannot be supplied, report the evidence gap rather than extending replay with a numerically converged but untrusted observation.
