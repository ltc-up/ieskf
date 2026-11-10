# Same native-time engineering comparison; no certified truth

All primary single-lap groups use the fixed historical common1197/1468 sample mask (81.54%), fixed AS tangent/segments, map/base rear reference and AT full-attitude antenna conversion. No GNSS enters filters. OldEKF is a recorded engineering comparator. Physical antenna installation,75ms delay, map datum and independent INS attitude are not certified; all accuracy differences are CONDITIONAL.

|Group|XY RMSE m|Long P50|P95|max m|Lat P50|P95|max m|Yaw engineering P95 deg|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|A NDT Fitness|0.577|0.328|1.017|1.908|0.145|0.493|0.705|3.811|
|B Native historical|0.631|0.316|0.965|1.771|0.262|0.679|0.842|3.866|
|C GICP fixedR|0.596|0.318|0.913|1.785|0.206|0.643|0.872|3.742|
|sameR NDT control|0.612|0.326|0.969|1.782|0.238|0.607|0.856|3.779|
|E historical oldEKF|1.137|0.687|2.027|2.539|0.239|0.713|1.109|3.997|


|Fixed segment|A NDT Fitness lateralP95|B Native|C GICP fixedR|NDT sameR|E oldEKF m|
|---|---:|---:|---:|---:|---:|
|0|0.430|0.479|0.514|0.483|0.523|
|1|0.337|0.513|0.521|0.478|0.796|
|2|0.220|0.390|0.342|0.314|0.538|
|3|0.674|0.723|0.833|0.831|0.420|
|4|0.446|0.614|0.518|0.557|0.616|
|5|0.414|0.607|0.599|0.562|1.065|
|6|0.075|0.097|0.222|0.110|0.188|
|7|0.245|0.303|0.345|0.310|0.281|
|8|0.344|0.752|0.514|0.605|0.616|
|9|0.668|0.831|0.737|0.822|0.270|


GICP's posterior worsens both difficult3/9 and all fixed normal-segment lateralP95 relative to originalA. It also loses primary globalXY. Against the **same fixedR** NDT control, GICP's XY improves0.612→0.596m, but not uniformly across directions/corners; it remains worse than originalFitnessA. Therefore the R difference confounds a naive backend verdict. Do not declare the matcher intrinsically worse solely from A/C, or better from one corner/raw pose.

|Reference assignment|A NDT Fitness|B Native|C GICP|NDT sameR|E oldEKF XY RMSE m|
|---|---:|---:|---:|---:|---:|
|0ms|3.428|3.279|3.265|3.306|2.239|
|75ms|0.577|0.631|0.596|0.612|1.137|
|150ms|2.677|2.880|2.881|2.841|3.974|


Every preset hypothesis is retained, none selected for promotion. Historical Native's previously reported0.628 versus0.573 values used another evaluation sampling convention; current0.631/0.577 are consistent native-time/common-mask comparisons, not a redesignedNative method. Z differs about−29m because datum/reference remains unqualified; Yaw/velocity/height are engineering differences/diagnostics, not certified accuracies.

D (new adaptive fusion) is NOT_IMPLEMENTED because AW1/AW2 do not justify a statistical direction-to-noise mapping. AV's mixed improvements are not a default or an additional successful method. Full individual-corner P50/P95/max/longitudinal/Yaw distributions, update fractions and all time assumptions are in the aggregate JSON files; failed/nonaccepted frames are retained, not erased from coverage.

Conditional historical visual-box clearance: GICP minimum0.548m, NDT original0.440m, oldEKF0.139m;0 modeled containment failures in each single-lap test. This is not actual tyre/vehicle safety certification and cannot outweigh worse reference error. Private full track/box/corner plots remain in ignored evaluation directories.
