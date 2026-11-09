# Mixed actual feedback outcome; full repair FAILED

Primary frozen75ms historical common mask: V3 XY RMSE0.590507m versus legacy0.577097m (+2.3%); B0 single-use alone0.594154m. Difficult right3/9 improve, but normal regions/globalXY degrade. Passing arithmetic, raising gain, or completing all matches cannot override the frozen screen. No full-fix/default promotion follows.

|GNSS assignment hypothesis|Legacy XY RMSE m|V3 XY RMSE m|Old EKF XY RMSE m|
|---|---:|---:|---:|
|0ms|3.428|3.485|2.239|
|75ms|0.577|0.591|1.137|
|150ms|2.677|2.608|3.974|


All three hypotheses are retained; they are reference assumptions, not calibrated physical delays. GNSS is never fused, fitted, used to tune covariance/prior or select the best offset. AT's full-attitude antenna-to-rear conversion is reused. Actual antenna installation, physical delay, map datum/altitude and independent attitude/truth remain unqualified. Z reference differs about−29m; this is not certified height error. Yaw/longitudinal/lateral/Z difference distributions and coverage are in JSON, not promoted to certified accuracy.

The extended raw GPS1 reference uses the same frozen AT conversion/protocol mask beyond the historical220s CSV. It is separately labelled `raw_extended_reference`; historical1197-mask metrics and raw1204-mask first-lap metrics must not be mixed. The latter explains0.592 versus0.591, without fitting a delay. It permits conditional evaluation of later preselected laps, not creation of certified truth.

|Preselected lap|Processed frames|Raw-reference valid|V3 XY RMSE m|Legacy XY RMSE m|Old EKF XY RMSE m|
|---|---:|---:|---:|---:|---:|
|1|1468|1204|0.592|0.578|1.138|
|2|1352|1235|0.666|0.626|1.479|
|3 prefix|638|568|0.517|0.422|1.909|


This is one continuous candidate run, no per-lap reinitialization. Third lap contains only638/1314 frames;3458/4134 overall (83.65%), interrupted at172.850s by cache/backlog. Lap2/third-prefix use unselected-in-time data but strongly correlated same-track/sensor inputs, not independent statistical calibration. Candidate globalXY worsens in both: no robust out-of-sample superiority. No completed three-lap claim.

Single-lap center matches1468/1468 converge, all accepted;55 use rank4 common combinations and1413 full rank6. Continuous prefix3458/3458 converge/accept,113 partial. These are conditional optimization/observation events, not known-correct-peak counts. Bounded probes can discover local competing basins; common wrong peaks, incomplete hypothesis sets and selection uncertainty remain possible. Wrong-peak precision/recall, missing-map robustness and calibrated NIS/NEES remain NOT_VALIDATED.

V3 lap P minimum eigenvalue5.56e−7, prefix5.01e−7; all saved P symmetric. Lap Z span10.311m, vz−2.151..2.140m/s; prefix Z span10.340m, vz−2.179..2.268m/s. No freezingZ/cross removal/Bias clipping. These bounded diagnostics do not establish height/velocity accuracy. Lap maximum wheelZ correction3.321mm, NDTZ correction0.513m, NDT velocity-change norm0.398m/s. Full Bias/state aggregates are retained; no numeric divergence observed before scheduling failure.

Conditional historical visual-box minimum clearance: candidate0.399m, legacy0.440m, oldEKF0.139m,0 modeled non-contained frames. Candidate clearance worsens relative to legacy. Historical visualization geometry is not certified car/tyre outline; reference/boundary survey error bounds absent. Physical tyre safety and production safety remain NOT_VALIDATED/NO-GO.
