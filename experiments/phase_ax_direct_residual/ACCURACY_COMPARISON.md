# Controlled accuracy comparison — engineering reference only

All methods use frozen loop44/map0105 and fixed native-time AT conversion. A/GICP-pose comparison reuses AW actual own-state runs; AX-A/B use new own-state direct residual runs. No GNSS is fused and no reference time/Mask/extrinsic is fitted. Current historical common1197 samples cover81.54% of the full lap. Report all processed frames and all rejections separately; reference coverage is not localization coverage.

|Full1468 lap,75ms convention|XY RMSE m|Qualification|
|---|---:|---|
|Original CUDA-NDT pose+ESKF|0.5771|historical actual baseline, CONDITIONAL reference|
|AW small_gicp pose+ESKF|0.5957|historical actual baseline, CONDITIONAL reference|
|AX-A GICP normal direct-IESKF|0.7778|actual full loop; replacement FAILED|
|Old EKF|1.1375|engineering comparison, not truth|
|AX-B native raw-moment surface|NOT_A_FULL_LAP|531/1468; cannot use its prefix RMSE as a full-lap score|

|Fixed segment|Original NDT pose lateralP95 m|AW GICP pose lateralP95 m|AX-A lateralP95 m|
|---|---:|---:|---:|
|0|0.4302|0.5145|0.6160|
|1|0.3366|0.5207|0.9611|
|2|0.2197|0.3424|0.4892|
|3|0.6740|0.8326|0.7965|
|4|0.4460|0.5179|0.6511|
|5|0.4137|0.5990|1.2191|
|6|0.0751|0.2220|0.3230|
|7|0.2453|0.3454|0.3915|
|8|0.3436|0.5135|1.6814|
|9|0.6675|0.7369|0.7936|

The two difficult corners and normal5/8 all worsen in AX-A. Three continuous laps do not rescue the claim. Under raw-reference masks, AX-A/original XY RMSE by lap:

|Lap|Reference samples|AX-A m|Original m|
|---|---:|---:|---:|
|1|1204|0.7767|0.5785|
|2|1235|1.0930|0.6255|
|3|1196|1.7508|0.5006|

All0/75/150ms overall XY/longitudinal/lateral/Yaw/Z and fixed segment metrics remain in A_lap_metrics.json/A_multilap_metrics.json. Z differences are unqualified datum differences, not height accuracy. Time sensitivity does not authorize choosing0ms when AX appears closer than the original. AW Native historicalB is0.631m on the same historical convention and is not a direct residual candidate.

AX-B has only531 processed frames,451 updates and326 common-reference samples; its failed-prefix75msXY RMSE16.275m is not comparable to a complete-lap score. Difficult3/9 occur outside its achieved prefix: NOT_EXECUTED, not improved. Its complete531-frame state history remains private with complete rejected tail.
