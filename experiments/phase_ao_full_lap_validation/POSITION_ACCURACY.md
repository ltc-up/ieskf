# Accuracy qualification
XY RMSE/P50/P95/P99/max, longitudinal/lateral error, Yaw error, Z error and velocity error: **NOT_VALIDATED**. No independently calibrated, time/reference-aligned truth system was established for the selected raw records. No error numbers are synthesized from two correlated estimators.

Raw VectorNav GPS/GPS2 and INS outputs may share hardware, corrections and information already used by the historical EKF. They are not independent truth. The map survey/datum, antenna point and timing contract also remain unresolved. No estimator-derived map alignment was fitted and then called truth.

Historical path length, endpoint distance and closed-traverse selection describe recording coverage only. They are not accuracy, a surveyed lap counter, or closure error against a fixed surveyed start/finish. Z spans are **model outputs**, not measured vehicle-height errors. Header/arrival correction differences are timing diagnostics, not position RMSE.

All attempted outputs, failures and time coverage are retained. Trusted global coverage is 0% on the executed manager streams; maximum continuous unanchored duration equals the processed interval. An error statistic over selected published samples would conceal this failure and is therefore not reported. No complete accuracy GO can be issued.
