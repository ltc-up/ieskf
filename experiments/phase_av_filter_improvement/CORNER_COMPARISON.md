# Actual single-lap fixed-corner comparison

All values are native-time **GNSS engineering-reference differences**, using the frozen75ms hypothesis and historical common mask1197/1468 (81.54%). Same AS/AT segment definitions/tangents; no recentering, alignment fitting, time selection or GNSS feedback. OldEKF is recorded historical output, not an independently rerun/qualified truth.

|Fixed segment|Legacy P50/P95/max m|V3 P50/P95/max m|Old EKF P50/P95/max m|
|---|---|---|---|
|0|0.114/0.430/0.602|0.112/0.478/0.733|0.190/0.523/0.664|
|1|0.133/0.337/0.350|0.164/0.244/0.257|0.687/0.796/0.814|
|2|0.089/0.220/0.238|0.042/0.132/0.152|0.355/0.538/0.578|
|3|0.182/0.674/0.705|0.158/0.495/0.549|0.173/0.420/0.452|
|4|0.222/0.446/0.462|0.058/0.375/0.385|0.498/0.616/0.626|
|5|0.273/0.414/0.432|0.268/0.460/0.475|0.599/1.065/1.109|
|6|0.054/0.075/0.079|0.051/0.075/0.080|0.178/0.188/0.189|
|7|0.176/0.245/0.253|0.171/0.249/0.257|0.220/0.281/0.287|
|8|0.259/0.344/0.361|0.193/0.387/0.417|0.307/0.616/0.678|
|9|0.468/0.668/0.678|0.357/0.552/0.562|0.183/0.270/0.278|


Right3 P95 improves26.6%, right9 improves17.3%, control1 improves27.5%. But segments0/5/6/7/8 worsen (including small changes); segment5 rises0.414→0.460m and8 rises0.344→0.387m. The predeclared no-other-corner-degradation test therefore FAILS, without post hoc tolerance relaxation. OldEKF remains better than V3 in the difficult right3/9 under this reference.

V3 full unit lateral effectiveK mean at right3/9 is0.1534/0.1475, versus AU original0.1032/0.1055. Mean native lateral innovation becomes-0.0621/-0.0347m, compared with original−0.152/−0.119m. Actual mean rear-reference lateral correction is-0.0113/-0.0044m. Smaller corrections after trajectory changes do not imply lower gain: residual history is already different. Full15D and attitude-lever terms are retained. The nonlinear mixture derivative is logged separately from effectiveK.

Restoring packet information does not multiply steady-state gain by5: repeated posterior shrinkage also lowers subsequent P. Right3/9 lateral prior standard-deviation diagnostics are0.0398/0.0436m. This reinforces that packet bookkeeping is not proof of calibrated P/R or a complete physical repair.

Private native-overlay/corner1/3/9 figures and full state histories are retained; public figures contain aggregate bars only. Adjacent frames are correlated, so no independent-sample significance or detection accuracy is asserted.
