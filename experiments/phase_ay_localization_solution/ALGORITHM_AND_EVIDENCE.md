# AY: add measured motion information before changing LiDAR gain

Baseline: `c647ac3224adee775d30dc6f9553c9b83612c060`. All original tracked motion/pose/AR/AS/AT/AU source files remain byte-identical. No production write, GNSS fusion, new map, Q multiplier, LiDAR R multiplier, hard zero-lateral constraint or cross-covariance deletion. Candidates are explicit opt-ins; main/defaults remain unchanged.

## Mechanism selection

AU already verified Kalman/Joseph/reset arithmetic. Its low lateral pose gain alone cannot identify which covariance is wrong. AV's increased LiDAR feedback improved difficult corners but regressed elsewhere; AW changed matcher without overall benefit; AX direct residuals retained shared-map/correspondence problems. Repeating these mechanisms would not supply missing physical information.

The production vehicle has an actual optical sensor measuring both longitudinal and lateral velocity. The frozen research baseline only assimilates scalar wheel forward velocity. An optical lateral measurement can distinguish actual sideslip from attitude/velocity/bias effects without imposing nonholonomic zero sideslip. This is a testable additional physical constraint, not proof that missing lateral sensing uniquely caused the historical corner error. Sensor installation, response delay and observation statistics remain conditional.

Read-only implementation evidence, production HEAD `aa50e35467f89b4963f882d1f5abaace46d0ce70`:

- `src/a2rl_message/eav25_bsu_msgs/msg/Kistler.msg`: distinguishes raw optical velocity from internal IMU-corrected velocity.
- `src/a2rl_vehicle/eav25_bsu/src/kistler.cpp`: Correvit CAN values converted from km/h to m/s. The actual loop44 bag contains the corresponding high-level measurement/status and raw CAN streams. The wheel input remains the verified old eav24 loop44 input; message package naming is not used to infer wheel protocol.
- `ekf_converter.cpp` Kistler callback: production consumes raw vx/vy with sensor status handling; this is an existing physical source rather than an invented lateral observation.
- `lever_arm_compensation.cpp`: sensor rotation and angular-rate lever correction are part of the original processing concept.
- Same-bag static TF supplies the optical sensor orientation/location; no GNSS fit or new guessed extrinsic. Private source evidence and TF values stay private.

Raw optical and its internally corrected counterpart must not both be assimilated as independent measurements. Neither NDT nor GNSS positions enter optical extraction, status selection or covariance generation. Runtime does not read any GNSS reference. Offline evaluation reads the frozen cache separately.

## Measurement model and full-state update

Original state order `[p_map_IMU, v_map_IMU, right-dtheta, bg_body, ba_body]`. R maps body to map. Let `r = t_body_sensor - t_body_IMU`, `R_bs` map sensor vectors into body, and E select measured sensor x/y axes:

```
u = R.T v + (omega_body - bg) × r
h = E R_bs.T u
H_v = E R_bs.T R.T
H_theta = E R_bs.T skew(R.T v)
H_bg = E R_bs.T skew(r)
H_p = H_ba = 0
```

The actual measured lateral value is generally nonzero. No sensor Z velocity is invented. Use full15D P, S, K, delta, Joseph covariance and existing right-SO(3) reset. Consequently optical updates can affect all correlated states; protection does not consist of deleting gain rows. Malformed/status-invalid/speed-mismatch/domain-invalid observations return unchanged state/P atomically.

One packet per fixed10ms Header bin limits use of a roughly250Hz stream, matching the original wheel cadence. Status must have actually arrived earlier, have stst0 and Header age at most250ms. This rate reduction does not prove independence; adjacent optical readings and map errors remain correlated. The status rule follows the existing production system and is an engineering qualification, not manufacturer certification of every status bit.

Explicit initial engineering R is diag(0.26,0.26)(m/s)^2: existing publisher0.01 plus the frozen0.5m/s speed-mismatch envelope squared. It was frozen before GNSS evaluation and never searched. Using the same conservative envelope for raw optical is a declared initial assumption, **not a calibrated optical noise model**. Cross-axis noise correlation and temporal correlation are not identified. The speed residual envelope and original physical update domains are retained, with no uncalibrated NIS rejection.

## Candidate sequence and changes

| Candidate | Actual change | Mechanism and outcome |
|---|---|---|
| L,15D | wheel x plus raw optical sensor y | Direct lateral motion evidence; improves difficult corners but negligible global improvement and segment8 regression. |
| XY,15D | raw optical x/y, suppress wheel x while accepted optical is fresh | Avoids double-assimilating two forward speed estimates; improves longitudinal/global and difficult-corner engineering differences; segment8 still regresses. |
| C,16D | XY plus a constant optical mounting yaw error beta | Tests whether fixed orientation/model mismatch explains remaining regression. Full augmented covariance, no reference fit; complexity does not remove regression or beat simplerXY overall. |

XY returns to wheel when the last accepted optical event is older than100ms. A newly rejected sample does not instantly remove a still-fresh accepted sample; this short coast is explicit. Source selection is replayed as part of the checkpoint metadata, including delayed insertions. No duplication of shared forward information while optical is active. Short dropout/timeout/duplicate semantics are unit-tested; the real runs do not establish severe hardware-fault robustness.

For C: `R_bs(beta)=R_bs0 Exp(ez beta)` and `H_beta = E[-skew(ez) R_bs(beta).T u]`. Beta is static, Q_beta=0, prior mean0/std1degree as an explicit uncalibrated installation hypothesis. Motion transition embeds original F and **G Q G.T** in16D. Wheel and NDT append a direct zero beta Jacobian but retain indirect cross-covariance updates. All16D states are jointly estimated; this is not a Schmidt filter or certified physical installation calibration. It can absorb map, clock and motion-model errors; no interpretive physical calibration claim is made.

Implementation files: `optical_velocity.py`, `optical_replay.py`, `optical_calibration.py`; runner/evaluator/probe are isolated in this phase. Existing AW Journal/evaluator/profiling support is restored verbatim from e15e7a2, with its source/license notice, rather than copying production code. Original AR CPU/CUDA binaries, source/target, crop/Deskew, FitnessR and fullSO(3) correction are reused. New predictions genuinely change Deskew/source/matching seeds and final poses.

## Why not the obvious alternatives?

Shrinking LiDAR R, inflating P or more elaborate matcher geometry would assume the unknown statistical scale or peak correctness. A segment-specific policy selected from GNSS error would leak evaluation into the algorithm. Changing sensor yaw/time by reference fitting would have the same problem. The chosen route supplies measured motion; its remaining failure is preserved rather than patched with geographical switches or parameter sweeps.
