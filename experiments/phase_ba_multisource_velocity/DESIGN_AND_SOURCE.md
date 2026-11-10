# BA: speed dynamics, common-error estimation and actual interventions

Original c647ac3, AY364d246, AZf25ec45, production aa50e354 and FAR-LIO ef205103 remain unchanged. No new dataset, GNSS fusion, map/crop/NDT100/FitnessR change, physical15D Q change or geographic policy. Production/stashes are read-only. New code is independent research code, not copied vendor/production implementation.

## What the production frontend actually does

`ekf_converter.cpp` takes Kistler raw opticalXY, passes each axis through a3-state polynomial frontend, queries using latest selected IMU time, selects wheel below1m/s and otherwise optical. Its published x/y covariance0.01 is a configured variance, distinct from the frontend's sigma_r=.01 whose **squared** value1e−4 enters its internal update. EKFModule consumes vx/wz, not both opticalXY. The original research15D consumes wheelX; AY directly uses raw sensorXY. BA's polynomialXY experiment therefore does not claim to reproduce the full6-state production backend.

`velocity_sensor_filter.hpp` uses X=[velocity,first derivative,second derivative], predicts with constant second derivative and queries a quadratic at t_query. Push is callback-driven (recorded optical median interval4.007ms, about250Hz); output/query is tied to the surrounding converter, not a fixed declared digital cutoff. `get_velocity` invalidates after1s but permits negative dt; initialization resets means while its covariance is not explicitly reinitialized by that source branch. BA rejects future-contributing samples and stale queries and resets both frontend mean/P on a>1s gap. It does not treat this as a certified hardware delay correction.

C++ integer expressions1/20,1/8,1/6,etc. evaluate to0: effective legacyQ is diag(0,0,dt) times q. BA independently implements the standard continuous-white-driving-noise covariance:

```
F = [[1,dt,dt²/2],[0,1,dt],[0,0,1]]
Q = q [[dt⁵/20,dt⁴/8,dt³/6],
       [dt⁴/8, dt³/3,dt²/2],
       [dt³/6, dt²/2,dt]]
```

The driver acts on the third derivative of velocity (physical snap); q has units m²/s⁷, not calibrated accelerometer noise. q=1 and sigma_r=.01 are inherited source assumptions, not GNSS-optimized values. Joseph propagation is PSD-tested by independent numerical integration. At nominal250Hz, legacy/corrected steady responses are nearly identical: correctingQ alone cannot explain or solve the benchmark error. Synthetic10m/s instantaneous step overshoots about2.95m/s; this extreme probe is not a claim about an actual vehicle speed jump. Ramp tests and frequency response are retained.

Latest optical-by-recorded-receipt at Bosch receipt gives negative query-delta proxy in about5.53% of41714 pairs (minimum−12.732ms). It is a **callback/timer proxy**, not proof of that fraction of actual production calls using future physical information. BA's output is explicitly a model estimate at the event Header, computed only from raw samples with earlier/equal Header and receipt. Same-time query semantics do not imply zero phase delay; response/model error and clock uncertainty remain.

## Actual implemented candidates

### F: corrected causal polynomialXY

Two sensor-coordinate frontends process every raw sample; one100Hz selected event carries current-time estimatedXY, actual query/contributing Header identities and conditional output covariance. Full15D installed sensor/IMU lever H/Joseph/reset are unchanged. R_out = original conservative engineeringR floor + conditional frontend P_vv. This is a new conditional uncertainty interpretation, not pretending a smoothed estimate is an independent raw measurement with identical statistics. Overlapping frontend histories remain correlated; this approximation is explicitly uncalibrated. Raw and filtered values are never independently fused twice.

### B: own-state deceleration-X intervention

Outside `a_body_IMU_x < −1m/s²`, keep AY rawXY and R. Inside, only X uses the causal polynomial estimate and its conditional variance; Y remains raw. Acceleration uses the current branch's propagated R,ba and held Bosch force, including gravity. It is a **deceleration proxy at the IMU**, not an independently certified brake-force label. No corner number, GNSS or frozen trajectory controls the gate. This deliberately narrow actual intervention tests whether speed processing during deceleration repairs subsequent state feedback.

A new raw-input guard preserves the original5m/s physical mismatch envelope before smoothing: an abnormal raw velocity cannot become apparently valid merely because its filtered mean changes slowly. There is still a filtered-update gate and full atomic state/P handling. Original normal B/B_multilap raw innovations stay below2.312m/s; the extra guard is inactive there. Final guarded1468 replay exactly matches seeds/P/NDT/K/delta of unguarded B. Fault injection corrupts raw samples **before** preprocessing, not just a postfiltered result.

### J: joint shared optical error, physical15D+2

State `[physical15D, b_optical_xy]`; h_optical = original installed-axis velocity model + b, H appends I₂. Constant b has initial mean0/P=.25I (existing0.5m/s common-error envelope), Q_b=0. Full17D cross-covariance, injection/Joseph/reset and pose/wheel updates retained. No bias clipping or physical15D P inflation. Bias is a nuisance estimate, not physical calibration.

Only250 raw packets **before first lap** estimate a conditional white-noise proxy: MAD(second difference)/sqrt6, with PSD robust cross-axis correlation. Estimated variances≈.164/.274(m/s)² and nonzero cross term differ greatly from the source frontend's1e−4 assumption. True high-frequency dynamics, quantization and temporal correlations prevent calling this calibrated sensor noise.

Raw optical and wheel enter jointly outside own-state hysteretic deceleration (enter−1/exit−.5m/s²); in deceleration wheel is omitted while optical remains fresh, otherwise wheel is available as fallback. When both sources are eligible, the fixed2× marginal covariance bound follows `R_joint <= 2 blockdiag(Rw,Ro)` for arbitrary cross-source correlation and **correct marginals at a common linearization**. It is not arbitrary gain tuning or a guarantee of complete asynchronous/cross-time statistical consistency. Existing state/common gyro and map errors are not all identified. This coherent joint model was tested and failed actual performance; estimated bias alone is not a cure.

### C: non-overlapping raw blocks

Each50ms block uses disjoint raw IDs once, fits degree2 velocity, and forecasts at the first next-block sample's actual Header; that new sample is reserved for the next block. No future sample enters the current fit. The50ms span is the LiDAR period, not a fitted corner scale. Three IRLS passes use a3-sigma metric from pre-run noise proxy. Conditional fit covariance plus inherited .25 common-error floor prevents unlimited confidence from dense repeated points; empirical residual scatter can conservatively increase it. Queries more than10ms beyond the last contributing sample are rejected/skipped. No claim that raw-ID disjointness solves all shared/cross-block errors. This candidate also failed actual lap performance.

### W: one-factor wheel-information diagnostic

OriginalA, with wheel updates omitted only when its own body-IMU acceleration<−1m/s². No replacement optical or LiDAR velocity, no parameter/R change; true own-state CUDA feedback. This is a causal diagnostic of the specified information removal, not an advocated source-selection product. It worsens overall reference error and right3, so the data do not support removing all deceleration wheel information as a solution.

## Other physical information, used honestly

- Current wheel producer selects a specific wheel with steering/yaw/radius conversion; bag source-version compatibility is not inferred from modern package names. Existing trusted loop44 scalar is retained. AZ's4wheel spread/braking disagreements remain risk evidence, not calibrated slip labels.
- Actual `/controller/brake` payload cannot deserialize with installed current schema; bag has no embedded message definition. BA records failure and does not guess pressure fields. Thus the IMU gate remains a defined deceleration proxy; “true actuation braking caused error” is not established.
- Actual auxiliary IMUs exist (Bosch≈200Hz,Kistler≈250Hz,VectorNav≈100Hz). Offline TF/rigid-body/time checks retain median/robust/RMS evidence; yaw correlations are high but acceleration differences, DBC sign overrides, differentiation noise and clock/lever contracts are unresolved. No evidence supports replacing Bosch or averaging sensors without separate biases/correlation.
- FAR-LIO `velocity_handler/derivative.hpp` computes SE3 log of successive registered poses divided by dt, with invalid/gap resets. BA computes same-source interval-average LiDAR velocity **for diagnostics only**. Those map poses already enter15D, so adding their derivative as an independent velocity update would duplicate information and introduce cross-frame noise correlation.
- Existing radar clouds really contain velocity/velocity_interval/SNR/confidence. A frozen first20s audit finds approximately40/53m/s ambiguity periods and80–95ms Header-to-record delays, with valid radar frame IDs. Units follow the read-only driver contract; scan reference time, sign/static-target identification, extrinsic qualification and multi-period de-aliasing are not validated. Unconditionally treating wrapped Doppler as ego-speed would be unsafe. This does not prove that radar lateral/partial constraints are impossible; no unqualified radar fusion or SDK copying is performed.
