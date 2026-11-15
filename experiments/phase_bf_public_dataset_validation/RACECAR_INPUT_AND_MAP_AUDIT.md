# RACECAR S3 unique candidate — metadata only

Selected future candidate: M-SOLO-FAST-100-140, LVMS S3, MIT-PITT-RW, nominal100–140mph category. Do not replace it after seeing errors. [Official repository](https://github.com/linklab-uva/RACECAR_DATA) and public S3 list were read on2026-11-15.

Exact objects:

- https://racecar-dataset.s3.us-west-2.amazonaws.com/RACECAR-ROS2/S3/M-SOLO-FAST-100-140/M-SOLO-FAST-100-140.db3 —20711575552bytes (19.289GiB), not downloaded.
- https://racecar-dataset.s3.us-west-2.amazonaws.com/RACECAR-ROS2/S3/M-SOLO-FAST-100-140/metadata.yaml —16132bytes, metadata reviewed.

CONFIRMED by metadata: storage sqlite3, compression_format/mode empty, duration899.997582975s,5,533,818messages. Front/left/right LiDAR counts11774/11868/11264 (about13.08/13.19/12.52Hz averages, not proof of regular sampling). Top/bottom rawimux112498/112499 (~125Hz). Top/bottom bestpos18000 each (~20 Hz); heading2=900each (~1Hz); local_odometry17999. No wheel-speed topic is listed. rawimu, corrimu, inspva/inspvax and insstdev counts zero. No recordedTF topic in this metadata; use declared official URDF only after frame validation.

The alternative E S3 bag is990986383360bytes, rejected at selection stage for footprint, not localization performance. Its metadata was reviewed but payload not acquired. No nuScenes duplicate or other session selected.

## Source hazards

The official README describes receiver axis/installation rotations and warns that local_odometry is already processed. It is not valid to apply the same rotations to every channel or treat this processed output as independent GNSS-free inertial input.

CONFIRMED source defect in official racecar_utils/src/convert_imu_msg.cpp (GitHub tree identity retained privately): linear_acceleration.x is assigned rawimu_in->angular_velocity.y. This helper subscribes RAWIMU, whereas the selected bag contains zeroRAWIMU and nonzero sensor_msgs/Imu rawimux. Therefore the bug is not proof that rawimux is wrong; rawimux producer, acceleration units/gravity, axes, header stamp versus bag stamp and scale remain UNKNOWN. Do not run this helper or copy its formula. Inspect actual rawimux values/source after separate approved download, without mutating original data or production converter.

## Reference/map limitations

BESTPOS is at antenna phase center. Require receiver/antenna/IMU/body extrinsics and timestamp alignment before comparison to filterIMU or declared rear-axle base. Heading2 alone is not a full6D attitude reference; INS topic absence blocks assuming trustworthy roll/pitch/Vz truth. Satellite/FIX state is not absolute integrity evidence. local_odometry may contain GNSS and must be excluded from online seed/weights/fusion when claimingGNSS-free.

No independent PCD map is verified. Hypothesis: map-only initial completed lap with reference-assisted mapping, all later laps test, conditional on actuallap identity/coverage and declared transform. Freeze before matching, never evaluate mapping scans as independent test. If independent map/reference cannot be established, label conditional functional validation, not absolute accuracy/NEES. This is one15-minute scene, no new scene download allowed.

Banked LVMS corners challenge map-Z zero velocity; A1/A2 remainShadow, noC fusion. 20 Hz50 ms is a compute target despite the bag's irregular approximately13HzLidar delivery. No payload, new matcher, adapter or replay executed. Separate RACECAR approval must follow KITTI completion and freshdisk audit.
