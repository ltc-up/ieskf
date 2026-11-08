# AP frozen pre-execution plan
AO baseline is 3667c9c6da84b82c1d9e4676b38436ed9f8f6da1, verified pushed before AP execution. AO is NO-GO. AP scope is limited mechanism diagnosis and declared injection, not robustness acceptance relative to a qualified normal system.

No AO source, state, interface, Q/R, configuration, gate, delay horizon, candidate selection, GNSS qualification or Map/Odom strategy changes. Freeze hashes are recorded before experiments. Fault runners may manipulate their private copies of observations only.

Input selection before AP execution:
- Whole loop44 real raw observation timing and its entire 917.07 s valid local trace; no use of only a known favorable14.7s fragment. Whole loop7/loop8/loop35 failures remain AO real evidence, not new tuning datasets.
- Existing five predetermined AM frozen Source/Target geometry frames 0,71,142,213,283 and stored seven-start GPU results; historical match endpoints are diagnostic reference poses, not truth or new causal predictions.

Injected cases against the same no-fault input: GNSS absence during the middle third, all GNSS delayed600ms, GNSS LLA latitude offset0.001degree, first NDT translation +100m X/+50m Z, first NDT duplicate, first NDT delayed600ms, first NDT two explicitly supplied hypotheses10m apart, middle-third NDT absence. First and middle-third selection depends only on time/index, never estimator error. Raw GNSS still remains unqualified: absence/delay/outlier injection cannot validate fusion or real recovery when the control has no anchors.

Map fault: remove a20m XY-radius cylinder from the private Target centered on the first recorded diagnostic candidate. Apply unchanged MapSupport diagnostics to all five fixed Sources at the same stored pose. This tests sensitivity of support metrics, not registered pose error. No covariance and no new registration is inferred.

LiDAR subsets front/left/right pairs require original per-source preprocessing and Deskew recomputation. Do not replace these with random deletion from fused Source. If frontend contracts are unavailable, record NOT_VALIDATED for each combination.

Check full local trace hash before/after all global fault runs, separate raw vs injected vs historical computed evidence, record every output/decision count. No wrong-peak precision/recall without labels; no XY RMSE without independent truth; no live20Hz claim. Preserve any failed runner invocation and its repair if needed.
