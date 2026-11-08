# Phase AL test plan

Pinned parent5ecc1c07872c94b56295f56d073f5052b3a6c081. Implement candidate contract, bounded timestamped history, SE(3) interpolation, shadow composition and ROS transport without a local-state/P write API. Reuse frozen full AA/AJ window, not a selected successful subset. Use frozen original config; verify every Source/time/map identity. Local15D is replayed with IMU/wheel only, common historical initialization disclosed.

Automatic tests cover interpolation, transform legality, map/extrinsic/reference/epoch/direction mismatch, duplicate/stale/future/rollback, gaps/bounds, backend/convergence unknown and wrong-global/local-isolation. Real control and shadow replays must match all1470 full state/P outputs bitwise. Shadow candidate consumption and publication are different events. ROS transport must run in an isolated domain/namespace and publish no TF. No accuracy or statistical acceptance claimed.
