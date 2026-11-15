# KITTI input and map audit — pre-download

CONFIRMED: 77 synced scans and OXTS samples; 846 extract OXTS records by official ZIP filenames. [Geiger, Lenz, Stiller, Urtasun, “Vision meets Robotics: The KITTI Dataset”, IJRR2013](https://www.cvlibs.net/publications/Geiger2013IJRR.pdf) describes HDL-64E and100 Hz OXTS RT3003 acquisition, body xyz and tangent flu channels, start/end/forward-facing timestamps, and documented communication outages with interpolation flags. These general specifications do not prove the chosen scene has no missing data.

The official point format is XYZ+reflectance, without an explicit per-point time column. Spin start/end and forward trigger timestamps are not individual firing times. Do not fabricate exact firing times from azimuth; a future approximate undistortion must be explicitly labeled and equal in both groups. Synced/processed and raw/extract point representations differ; do not assume raw motion distortion or pre-compensation state until the devkit/source is checked.

## Adapter acceptance checklist (all not yet run)

- Preserve full date/time precision as int64; evaluate monotonicity, interval distribution, missing/interpolated samples and common timestamp coverage; index is not timestamp.
- Use extract OXTS body acceleration/angular rates, not world/flu acceleration, not navigation speed as a wheel surrogate. Verify whether exported acceleration includes gravity before applying the15D gravity model. If not compatible with specific force, stop rather than double-compensate.
- T_velo_imu from official calibration, pointcloud frame and filter IMU reference must be explicit; never reuse private racecar extrinsics. Predict p_IMU/v_IMU with right SO(3) error. Transform map observations at LiDAR or declared base frame with the corresponding lever arm exactly once.
- Trace record availability time separately from scan epoch and NDT completion; do not fuse delayed observations as current. A ROS2 bag conversion is optional, not a prerequisite to correctness; no giant duplicate bag required.
- Wheel and Kistler disabled for both groups. No online GNSS correction, GNSS velocity update, GT attitude C, GNSS-derived map-prior reseeding or weights. One declared initial orientation/position (and velocity if unavoidable) may use reference, which must be labeled reference-initialized.
- OXTS orientation/velocity/position form a coupled GNSS/INS solution. Evaluation is a reference comparison with shared inertial errors, not independent absolute truth or full15D NEES.

## Map and test contract

Proposed split fixed before any matching: sync scans0–19 for map,20–76 for test. Use only map-side reference poses to place map points, separately acknowledge map reference dependence. A short forward drive may have little heldout field-of-view overlap. No independent map has been found; do not claim generalization or full-sequence accuracy if building a map from all test poses. If coverage fails, retain failed matched samples and classify function-only/insufficient-map. No second scene permitted.

## Version contract

The public research baseline is da3ea9df. Its native_pose_R transports full covariance using explicit qualified PCM→map and internalRxRyRz parameters; BE49 same-map certificate does not apply to KITTI. Each new adapter must qualify mean/chart/reference-point transformations. The actual private CUDA wrapper, binary/source identity, invocation parameters and legacy policy must be captured and frozen before comparison. Historicalc647 OFF and aa50 GLL3 are different versions; no optimizer change or assumption about deployed lidar_ndt/v1 is allowed.

No adapter or matching has been executed in BF0. All runtime frames, gravity/units, true sampling intervals and valid map coverage are UNKNOWN.
