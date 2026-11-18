# B local odometry

Let U_i be independently propagated IMU/Wheel base pose with no LiDAR updates. The common causal replay initializes its original mean, biases and P from the authorized 135 s initialization and complete 130 s prefix. This adapter consumes U_i and cannot supply a missing prefix or certify the caller's initialization.

At first reference time choose fixed O_T_U with initial translation removed and initial yaw removed, preserving gravity alignment. O_0 = O_T_U U_0. For each subsequent frame:

ΔU_i = inverse(U_(i-1)) U_i; O_i^- = O_(i-1) ΔU_i.

The native CUDA matcher proposes O_i^L from same Source versus historical real Target. If the engineering validator accepts it, O_i = O_i^L; otherwise O_i = O_i^-. U_i remains independent in both cases. This is a continuous Pose chain with motion prediction, not a LiDAR update to velocity, biases or complete 15D covariance. B-P uses only ΔU, bypassing Target assembly and GPU.

The frozen engineering rule requires converged/candidate_valid, ≥20 valid Gaussian Target voxels, ≥100 distinct neighbor-supported source points and ≥10% source support. Translation innovation relative O_i^- must not exceed 1 m and full rotation innovation 5 degrees, multiplied by min(2,actual_ref_dt/.05 s). Multiple Gaussian pairs are never counted as separate supporting Source points. It checks numerical finite/rigid output. Rejected proposals and reasons are preserved; they never advance the local pose with a rejected transformation.

After failure, B0 stores THIS real frame and O_i^- as the next anchor. This avoids changing the requested previous-frame algorithm into a last-accepted submap. Repeated failures can cause predicted anchor drift; neither smoothness, high support nor local positive Hessian resolves shared ambiguity. No statistical high-confidence claim is made. Large useful corrections may be rejected by the engineering domain; report accepted/rejected innovations rather than claiming a correct-peak classifier.

B-M stores up to three previous frames within one second. B-K has the same explicit bounded history budget but inserts when joint occupancy overlap ≤.7, removes <.01 support after exceeding capacity and removes the lowest latest-support times nonredundancy score. These settings are frozen engineering comparators, not production-tuned GLIM performance claims.
