# Selection before algorithm execution
No accessible input yet satisfies all normal-lap and independent-reference conditions.

| Input | Input-only evidence | Qualification / reason |
|---|---|---|
| loop35 raw SQLite | 757.618 s; all three LiDAR ~15,150 each; Bosch 151,471; scalar wheel 75,754; two GNSS ~3,789 each | Primary candidate; every scalar-wheel Header is zero. No substitution of recorder time. Historical pose has a return after 245 s / 3 km, not surveyed lap certification. Local wheel fusion is blocked. |
| original loop7 | 1433.481 s, three LiDAR 28,666 each, Bosch 285,898, wheel 142,459, GNSS 7167 each | Backup raw input; runtime/map/TF/time/reference qualification required. |
| original loop8 | 2467.520 s, three LiDAR ~49,349 each, Bosch 490,967, wheel 246,751 | Alternate backup; metadata alone does not establish normality. |
| loop7-after-940 | 493.481 s MCAP; Bosch 98,626; wheel 48,463; Kistler twist 123,321 | Raw extraction executed. Historical pose only 2.141 km, no closed traverse; pose gap 52.859 s, NDT gap 51.950 s, wheel gap 8.871 s; no recorded static TF. Cannot supply qualified full-lap local/global run. |
| R2 exported Source fixture | 2292 clouds, 116.793 s pose coverage; recorded pose path 5.294 km, endpoint 572.13 m from start | Not a complete-lap certificate. No Bosch/Kistler/wheel streams. Fused odom already globally corrected. Use timing fixture only. |
| loop44 | 917.804 s raw SQLite, all sensors | Known historical faults; limited whole-record execution fallback, never qualified normal baseline. |
| other T7 loop2–6,9–10 | Inventory retained privately; all have nominal sensor counts | Not chosen over primary/backup based on estimator results; no independent reference/map contract established. |
| test_loop7 CPU/GPU exports | Pose output bags | Not independent raw sensor/control baseline. |

Full payloads and absolute timestamps stay private. Source counts do not establish LiDAR health, map coverage, independence or point acquisition timing. No normal-lap accuracy truth has been found. New algorithm results cannot revise this selection contract.

## Later reference audit, disclosed chronology
A pre-existing `nmls_full_lap` GNSS-derived artifact was discovered after whole-record loop44 execution. It names a 73.4345 s lap window and a derived CSV with 75 ms delay correction. It is audited as additional historical coverage evidence; it does not retroactively change primary/backup selection or confer independent truth. The whole continuous local trace covers this window without a reset, but its Z behavior and zero global coverage fail eligibility. No algorithm-error-selected window is used to announce success. See `reference_audit.json` and `prior_lap_results.json`.

Original loop8 backup was then fully extracted: 246,751 wheel, 490,967 Bosch, 613,904 Kistler twist, 47,560 recorded NDT poses, 246,740 historical pose outputs and 12,338 GNSS messages per receiver. Wheel/IMU Headers are nonzero; maximum internal IMU interval 6.219 ms, wheel interval 15.546 ms. Static TF is recorded. Its requested whole-record local attempt is retained even though the resulting state is unusable; this input was not selected because of a good estimator result. Acquisition/reference/map/independent-truth qualifications remain incomplete.
