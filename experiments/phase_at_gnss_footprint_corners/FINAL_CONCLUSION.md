# Phase AT conclusion

Baseline fe55fd3113616ea471373bc68bfbc23dc4596a45. **No estimator, Q/R, NDT, GNSS fusion or production modifications.** Existing AR/AS transactions are preserved. Actual offline processing uses1468 frozen real CUDA closed-loop transactions plus91778 raw attitude and4589 samples per GPS source. This phase does not claim another live/GPU localization run.

| Question | Qualification | Finding |
|---|---|---|
| Existing GNSS conversion/lever implementation | VERIFIED numerically |3770 valid rows reproduced to3.23μm maximum; full attitude and rear-axle conversion already existed |
| Actual antenna installation and phase centre | NOT_VALIDATED |bag lacks antenna TF; historical-copy candidate is not a survey |
| GNSS physical time/75ms | CONDITIONAL |native raw reconstruction and shifted-curve experiments differ; fixed delay hypotheses reverse ranking |
| Height datum | CONDITIONAL |documented reference-origin difference accounts for28.46m; residual−.561m and actual map datum remain unresolved |
| Engineering single-lap XY | CONDITIONAL |75ms raw reconstruction new/oldRMSE .579/1.138m; not certified true accuracy |
| Visual-box footprint geometry | VERIFIED computation, CONDITIONAL model |1468/1468 new/old and1197GNSS tested, no modeled boundary intersections; minimum .440/.139/.619m |
| Actual body/tyre line crossing | NOT_VALIDATED |historical measured outline, tyre edges, dynamic steering/contact TF and surveyed drivable boundary/error bounds missing |
| Fixed right-corner formation | VERIFIED decomposition, CONDITIONAL error |prior P95 .721/.686m; NDT .286/.437m; posterior .674/.668m in3/9. Current update mostly reduces prior discrepancy |
| Physical root cause / correct NDT peak | NOT_VALIDATED / UNKNOWN |time/installation/reference yaw and upstream prediction cannot yet be independently separated |
| Regression | VERIFIED |171 passed,0 failed;11 new directed tests |
| Safety/production release | NOT_VALIDATED; NO-GO for release |positive modeled clearance does not bound real tyre safety; no online qualification added |

The right-turn discrepancy is not explained by an omitted antenna lever or the present NDT update alone. Evidence supports examining prediction/reference time alignment and independently measured installation before altering filter weights. No new R, fitted offset, fitted lever, optimized delay or fabricated tyre dimensions were introduced.

Full private native-time state/gain/reference arrays, polygon records and corner plots are retained in the ignored durable AT directory. Public figures show aggregates only. Read GNSS_CALIBRATION_AUDIT, TF_VEHICLE_GEOMETRY, TRACK_FOOTPRINT_REPORT and CORNER_CAUSAL_ANALYSIS for assumptions and limits. Multi-initialization matching and interframe tyre/body sweeps were NOT_EXECUTED because their required labels/geometry were not qualified.
