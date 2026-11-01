# Candidate evaluation using surface distances

Executed: saved prediction, AD G1/G2 coordinates and AA first committed partial coordinates, frozen same Source/Target/time/P. No AD objective or Hessian used to score, weight or compute backend covariance. Surface extraction is outcome-independent after declaring the union of these four query regions plus a cell halo. It is therefore not a held-out validation region.

|Candidate|Association|Source residuals|median abs m|P90 abs m|P99 abs m|
|---|---|---:|---:|---:|---:|
|prediction|prediction_frozen|1368|0.712423|1.032324|1.418850|
|prediction|own_selected|1368|0.712423|1.032324|1.418850|
|G1|prediction_frozen|1368|0.139745|0.648624|1.556579|
|G1|own_selected|2069|0.132732|0.509306|1.414550|
|G2|prediction_frozen|1368|0.143288|0.951217|1.751074|
|G2|own_selected|1433|0.133582|0.807234|1.298474|
|AA_partial|prediction_frozen|1368|0.145198|0.651392|1.596553|
|AA_partial|own_selected|1883|0.177915|0.656979|1.504106|

Prediction-frozen columns use the SAME plane factors and rows; own-selected columns use different supports. Own totals cannot be ranked as the same objective. Signed residuals/full factors and identities remain local; public file contains only authorized aggregates/spectra. Same Source from candidate generation is reused; plane fits overlap AD map neighborhoods. This is frontend/backend functional separation, not statistically independent candidate validation.

G1 has lower P90 surface residual than G2 in both views. Neither has an independent absolute pose reference. AA partial can score lower shared quadratic on common factors, without proving it is correct or converged. Residual tails remain large; prediction has90.35% residuals exceeding epsilon std0.2m (descriptive, not a calibrated normality/gating test). Learned covariance, per-candidate R tuning and best-seed selection are not used.

New geometric H uses surface normals/lever arms only; tangential map density and support-kernel derivatives cannot directly supply information. A real varied-normal scene can still have rank6. G1/G2 cannot be credibly resolved into a unique correct pose from this contaminated one-scene evaluation. MULTIMODAL_CANDIDATE_VALIDITY=NOT_VALIDATED; no forced winner.
