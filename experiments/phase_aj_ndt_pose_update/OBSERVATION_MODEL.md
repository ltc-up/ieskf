# Observation and engineering-weight model

The full nominal state is [p_IMU_map,v_IMU_map,R_map_body,bg_body,ba_body], error [dp,dv,dtheta,dbg,dba]. Position/velocity increments are map-frame; attitude injection is right multiplicative R+=R*Exp(dtheta). No Euler attitude propagation is introduced.

With fixed lever l from base to IMU in body coordinates, p_base=p_IMU-R*l. The observation is h(x)=[p_base.x,p_base.y,atan2(R10,R00)]. Measurement z uses the same map-from-base NDT transform at the exact native Source reference time. Innovation is z-h; yaw innovation is atan2(sin(dyaw),cos(dyaw)). NDT Z/Roll/Pitch are not measurement coordinates. The transform's non-yaw coordinates never update an independent sensor channel.

The 3x15 H has position XY identity and attitude rows [R*skew(l)]_XY for the base lever. For u=R*e1, heading's right-chart derivative is [-u_y,u_x,0]/(u_x²+u_y²) * [-R*skew(e1)]. All other direct columns are zero. Singular heading charts are rejected. No approximation setting dtheta_z equal to heading error is used away from level pose.

K=P*H.T*solve(H*P*H.T+R_obs,I); delta=K*innovation. Joseph P_chart=(I-KH)P(I-KH).T+K*R_obs*K.T, followed once by the original right-chart reset B=chart_map(delta). Full15 cross terms remain. A finite, SPD, safe candidate is committed atomically; failure retains both prediction state and P. No point-plane factor is run and no same-Source double fusion occurs.

## Historical engineering R, not calibrated noise

Source/config audit confirms the legacy fitness covariance path, not native covariance publication. Let f=max(PCL fitness,1e-5). The raw six diagonals are translation=f and angle=f/2. The callback rejects invalid/out-of-range raw diagonals; for f<3 it scales translation by .05 and angle by .0005; otherwise by 1.5 and 1.2. The pose update selects XY/Yaw and multiplies by its configured smoothing count 5. Thus one C update uses:

- f<3: diag(.25f,.25f,.00125f).
- f>=3: diag(7.5f,7.5f,3f).

Fitness >15 is rejected by the historical raw-diagonal contract. Source publication's existing 1e-5 floor was confirmed in final audit and included in the public implementation. All observed finite fitness values are >=0.088709; all 153 saved R matrices are exactly unchanged by completing this unused edge case. It is not a tuned covariance floor or a new experiment. NaN/negative fitness is invalid. No native/Hessian covariance enters R. All 66 accepted NDT candidates also satisfy the historical native matrix finite/symmetry/positive-diagonal health check, without using that matrix as noise.

The legacy filter repeats queued observations five times with the inflated per-pass R. User-required C consumes each Source ONCE and retains that per-pass engineering R; it does not claim equal total frame information to the old five-pass queue. No R multiplier search, fitting, new NIS gate or calibration was performed. Published NIS is descriptive only: shared IMU/Deskew/map errors and this empirical R prevent a calibrated chi-square interpretation.

## Guard semantics

The original .5m translation and .05rad attitude *single-step* bounds are conservatively applied to the ONE Kalman proposal; original total2m/.15rad and gyro/accel bias limits .03rad/s/.3m/s² also remain. No velocity bound is invented. The .5m/.05rad limits originated as solver-step trust bounds, not physical hard-state constraints: this explicit safety interpretation can reject a large but otherwise legal one-shot measurement correction. Retention conclusions are conditional on this frozen policy, not a proof that every NDT-pose implementation must fail. No proposal is clipped or distributed over multiple updates. Wheel behavior is unchanged, including its separate original acceptance semantics.
