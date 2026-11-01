# Input and prior identity

Baseline AH commit: 262e22f625aa4b2410e5418429c2ed0bba27a432. Exactly the five predeclared full-deskew A events E1–E5 were reused. No new NDT, Deskew, map extraction, sensor replay or initialization search was performed. Native Source file hashes and the complete shared Target hash are in the manifest. Each frozen prior state time equals its native LiDAR reference time exactly.

State order is [position_map, velocity_map, right attitude error, gyro bias_body, accel bias_body]. The position state is the IMU origin. The stored NDT pose maps base points into map: R_seed=R_NDT, p_IMU=t_NDT+R_NDT*lever. Rotation.from_matrix projects backend float rotations to SO(3); translation is untouched. All pose errors use the original prediction and right perturbation. This projection is a representation conversion, not guard clipping.

Pose indices are [0,1,2,6,7,8]; remaining indices [3,4,5,9,10,11,12,13,14]. One completion policy was selected before the solves: other=solve(P_pose, P_pose_other).T*pose. It minimizes the original Gaussian prior at a given pose without consuming any measurement. The frozen mean and complete P never change. This is not the only possible completion; rejection under this policy does not prove that no other feasible completion exists. No alternate completion was searched in this phase.

All five NDT poses pass total pose guards. Completed E2 violates gyro-bias; E3 violates gyro- and accel-bias; E4 violates accel-bias. Only E1/E5 proceed from NDT seeds. These are engineering guard failures, not a proof of physical impossibility.

AH NDT necessarily consumes float32 native points; the original IESKF uses exact saved float64 native points. Point identity/order and geometry source are shared, but scalar bit identity across backend interfaces is not claimed: maximum component rounding is 3.80–3.814 micrometres. The IESKF Source was not recast to match NDT. All prediction-start decisions reproduce AH; accepted deltas agree below 1e-8. The historical accepted-state score for a rejected event refers to the unchanged prior, not its rejected last iterate.

Full state/P, motion histories, exact reference times, lever, Source/Target data, per-iteration trial states and correspondence identities stay in ignored local evidence. Public hashes identify inputs without publishing payloads. Original IMU/wheel/noise/Huber/geometry/guard configuration is unchanged.

