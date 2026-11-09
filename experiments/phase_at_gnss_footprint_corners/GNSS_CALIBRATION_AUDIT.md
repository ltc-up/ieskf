# AT1 — GNSS reference and timing audit

Status: **CONDITIONAL engineering reference; physical calibration NOT_VALIDATED**. AR/AS estimator, measurements, delays used by the estimator, Q/R and matching parameters are unchanged. GNSS is evaluation-only.

## Evidence chain

Read-only source roles (source hashes in manifest): `a2rl_vehicle/vectornav/src/vectornav.cc::getTimeStamp`, `vn_sensor_msgs.cc::{sub_vn_gps,convert_to_enu}`, configuration `vn_sensor_msgs.yaml`; `a2rl_launch/launch/urdf.launch.py`; historical project-copy `a2rl_race.urdf.xacro`; original archived `reconstruct_vectornav_gnss_truth.py`; original bag `/tf_static`.

The existing reference already uses GPS1, full quaternion conversion NED/FRD to ENU/FLU, attitude bias and the antenna→VectorNav ENU→base chain. It is not an uncompensated antenna trajectory. AT reuses GeographicLib CartConvert and the same declared origin; no trajectory registration or fitted lever is introduced.

Explicit equivalent equation is `p_map_base=p_map_ant−R_map_base*t_base_ant`. `T_base_ant` maps antenna coordinates into base; its translation is the antenna position in base. Antenna orientation is removed from the converted attitude to obtain base attitude. Pitch and roll participate. `antenna_to_base` has a nonplanar rotation test. The old chain's numerical reproduction agrees over 3770 valid historical rows: 3D maximum 3.23e−6 m, orientation maximum 2.40e−9 rad. This verifies implementation equivalence, not installation calibration.

Raw bag static TF records base→front axle, front axle→Bosch/VectorNav/LiDAR; it does **not** record antenna1/2 or vectornav_enu. Shared recorded/historical-copy translations and rear-axle/Bosch/VectorNav rotations agree; three LiDAR rotations differ (maximum .023rad), so recorded LiDAR transforms are retained rather than overwritten; missing antenna transforms are explicitly supplied from the historical project copy and remain UNCALIBRATED. Current launch selects `a2rl_${A2RL_MODE}.urdf.xacro`; historical environment and antenna phase-centre survey are unavailable. Current and historical GNSS configurations differ in child frame, attitude bias and position bias. AT uses the historically reconstructing configuration; no undocumented current bias is added.

## Map and height

GPS geodetic coordinates → LocalCartesian ENU at the declared historical origin → direct map-labelled pose → rotated antenna lever removal → rear-axis base. Neither a surveyed map↔ENU registration nor a recorded independent map↔ENU TF certifies identity. The publisher labels its ENU output `map`; that label alone is not a survey.

Source `lidar_ndt_v1/include/lidar_ndt/common.h::RTK_REF_POINT` uses a zero-height origin; historical GNSS LocalCartesian uses a different configured origin height. The documented origin difference accounts for about 28.46 m of the native-time mean Z discrepancy of −29.019 m. Applying **only this documented origin change**, without fitting, leaves mean −0.561 m (RMSE .579 m). This is a datum diagnostic, not corrected height truth. Map0105 acquisition datum, geoid/ellipsoid relation, actual runtime map-origin settings and residual cause remain NOT_VALIDATED. A −29 m discrepancy must not be labelled a proven geoid correction.

## Time

`getTimeStamp` returns host `now()` unless adjust_ros_timestamp and timeStartup are enabled; optional adjustment uses a moving host/sensor offset. Raw GPS and attitude share packet-derived Header stamping. Receipt-minus-Header P95 is about 1.01 ms for GPS1, 2.45 ms for GPS2 and 2.05 ms for attitude; these are recorder delays, **not** sensor measurement latency. INS orientation interpolation is bracketed, with ≤50 ms attitude gap; no future reference enters filtering.

Historical 75 ms is a reconstruction constant, not an independently demonstrated PPS/measurement-age calibration. Three predeclared hypotheses (0/75/150 ms) are all retained. A shifts queries on the already compensated curve; B shifts raw GPS measurement assignment, interpolates antenna coordinates, and uses attitude at the unchanged physical query time before lever compensation. They differ by up to about .13 m at P95 for the nonbaseline hypotheses. Native-time B XY RMSE (new/old): 0 ms 3.433/2.243 m; 75 ms .579/1.138 m; 150 ms 2.680/3.979 m. Ranking can reverse. No best-offset selection is a calibration.

## Validity and independence

Raw extraction: GPS1 4589, GPS2 4589, attitude 91778 samples. GPS1 fix8 count4227; GPS2 fix8 count3277. Publisher code explicitly maps8 to its RTK-fix case; actual receiver/firmware protocol certification is still missing. GPS2 is not substituted or assumed an independent receiver. The historical reference has 639 invalid rows; native-time evaluation accepts1197/1468 frames, no extrapolation and no interpolation across GNSS fixed gaps over approximately .6 s. GNSS/INS shared processing and any historical production-EKF GNSS usage lack independent-truth proof. XY and yaw are engineering-reference differences; Z accuracy, absolute yaw accuracy, true estimator superiority and safety bounds are NOT_VALIDATED.
