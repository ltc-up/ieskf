# Surface observation model

Executed: independent surface residuals, shared-latent effective noise, synthetic observability/copy checks, and one real frozen first-prediction diagnostic. Historical evidence: AD provides G1/G2 candidate coordinates; its energy, kernel forces and Laplace matrix are never read for observation scoring or covariance. Hypotheses and unexecuted items are separated below.

For one physical plane g use reference anchor c, unit normal n, tangent basis T (3x2), and local parameters eta_g=[alpha_1,alpha_2,d]. The first-order perturbed signed distance is

`r_i = n.T (y_i-c); H_i = dr_i/d delta_x; J_i = [(y_i-c).T T, -1]`.

`y_i = p_IMU + R_IMU (source_base_i - lever_base_to_IMU)` follows the existing research GeometryFactor and frozen extrinsic. Position/velocity are map coordinates, attitude uses right body perturbations, bias coordinates follow the original15D state. Direct H columns for velocity/bg/ba are zero; pose indices are[0,1,2,6,7,8]. Full15 cross-covariance is kept, so indirect corrections need not vanish.

Under EXPLICIT assumptions (eta_g Gaussian, independent across genuine physical planes; epsilon independent across distinct physical Source identities; both independent of prediction),

`r = H delta_x + J eta + epsilon`,
`R_eff = D + J Sigma_plane J.T`.

D uses unchanged research epsilon variance0.2² m². It is not a calibrated physical sensor law. Each plane uses one3D latent and full covariance, not one scalar per point. B=J chol(Sigma); Woodbury computes R_eff solves per group via3x3 systems. Explicit latent marginalization matches the15D effective-noise posterior. No large point-level innovation matrix is needed. Source aliases sharing measurement identity are canonicalized; inconsistent aliases reject. Target records are exact-coordinate deduplicated before extraction. A physical plane registry shares one latent only when identity is explicitly supplied.

Anchor change inside the same plane uses `eta_new = A eta_old`, with `A[2,:2]=-T.T shift`, and transported Sigma=A Sigma A.T. Resetting anchors while copying the same Sigma is generally NOT an invariant representation. This transport is separately tested.

Frozen reference update is linear Gaussian, not a reimplementation of the AD nonlinear Huber mixture. AD/old Huber parameters are unchanged and unused as observation information. No data-dependent Huber weights are slipped into R_eff; Gaussian residual validity on the real scene is NOT asserted. R_eff and its Jplane are frozen at the evaluated state. This is not a nonlinear likelihood optimization accounting for dR/dx or logdetR. It is a one-linearization conditional numerical interface.

Synthetic laws are declared generative: diagonal std[.01rad,.01rad,.1m] and a correlated Cholesky[[.01,0,0],[.002,.01,0],[.01,-.005,.1]]. Real use transfers only the latter as ONE uncalibrated scenario. No covariance fit/tuning was attempted. Actual map acquisition, normal/model/registration uncertainty and prior–Deskew–Source correlation remain unknown. The model must not be described as a calibrated map observation.
