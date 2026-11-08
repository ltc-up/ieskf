# Robustness comparison — no qualified baseline
AO is NO-GO; all AP comparisons are limited diagnostics. No accuracy/robustness improvement relative to the original production EKF is established. Historical production outputs have no qualified same-runtime map/config/acceptance reconstruction or independent reference.

Nine controls/fault schedules each evaluate91707 time slots on the same local trajectory. All have0% trusted coverage and917.07s maximum continuous lack of trusted localization. No real recovery time, meaningful coverage degradation relative to a GO baseline, independent position/Yaw error, or wrong-peak recall/precision is available. Zero false trusted outputs in an already unqualified all-rejected system is not a successful robust locator.

Executable behavior supported by evidence: duplicate identity rejection; stale GNSS reason; chronological rollback rejection; supplied two-mode retention; exact no-local-write boundary. The two-mode scenario has1 INITIALIZING,40 AMBIGUOUS and91666 UNAVAILABLE slots; control has1 INITIALIZING and91706 UNAVAILABLE. No GLOBAL_TRACKING, PARTIAL or COASTING is legitimately earned because no qualified anchor was ever acquired.

Map-hole injection measurably lowers one support ratio but not rank/classification. Historical GPU clustering retains two ambiguous groups; unknown true-peak labels prevent wrong-peak classification claims. GNSS cannot select modes while unqualified. No global pose is published, so global jump magnitude and recovery accuracy are NOT_APPLICABLE, not measured zero. The synthetic shadow test shows wrong global corrections remain possible despite exact local isolation.

Partial-LiDAR success, actual fault-time new NDT matching, GPU overhead and online20Hz remain NOT_VALIDATED. Severe pre-existing local vertical failures remain a separate NO-GO. Acceptance/rejection counters and performance distributions are in `fault_results.json`; unsuccessful probe prerequisites remain in FAILURES.
