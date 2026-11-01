# Independent research architecture

`InertialPredictor` propagates position, velocity, SO(3) attitude and two three-axis Bias states. Error order is `[dp,dv,dtheta,dbg,dba]`; position/velocity increments are world-frame and attitude is right-multiplicative. Configuration defines gravity/noise. Extrinsics must be explicitly supplied; no private vehicle geometry is distributed.

`WheelObservation` models longitudinal reference-point speed with an explicit lever arm. `GeometryCorrespondence` queries a static map and fits qualified local planes. `GeometryFactor` embeds the independent six-pose-DOF factor into the full state. `IteratedErrorStateUpdate` freezes a prediction prior, relinearizes geometry and performs one conditional IRLS covariance calculation; its historical iteration-limit partial acceptance is intentionally retained and unqualified.

`objective`, `partition`, `constrained` and `posterior` expose independent historical mathematical research functions. Hard-constrained states must not be interpreted as an unconstrained Gaussian posterior. The portable posterior module is a fixed-linearization numerical approximation, not calibrated uncertainty.

`EventReplay` sorts frozen physical/Header-time events and produces offline states. `InstrumentedUpdate` only observes proposals/trials/target IDs; it preserves all historical decisions. Diagnostic scripts distinguish raw proposals, committed changes, carried states and data-dependent map residuals. No ROS topics or TF are published.
