# BE5 queued after E0–E4

The user's additional vertical/attitude/covariance study is a subsequent supplement, not an alternative acceptance criterion for this phase. No BE5 observation, native-R retuning, optimizer change or production change is included here.

The next deliverable must first audit actual 15D input topics/adapters, node/launch identities, state chart, base/IMU reference points, pose covariance transport and delay. Mark evidence CONFIRMED / HYPOTHESIS / UNKNOWN. Then derive A (base-point vertical velocity soft observation with angular lever arm) and C (SO(3) roll/pitch reference soft observation, correlation/dynamic-acceleration limits). B road-height constraints are excluded.

Separately compare fixed-pose Legacy, Native and independently calibrated Native covariance. Native curvature is local GN/Fisher-like information, not proof of a correct branch or an automatically calibrated statistical covariance. Do not multiply Native by PCL Fitness; retain Fitness provisionally as a separate diagnostic pending an actual source/cost audit. Preserve full 6×6 cross terms and transform scaling as D R Dᵀ. No new multi-start, GICP, optimizer, line-search, Q/bias model or static-TF changes belong to BE5.

Before any BE5 code, submit minimal file/function/parameter changes, Jacobians, single-factor frozen-observation tests, combined A/C and covariance experiments, acceptance/failure criteria, and obtain the user's confirmation as explicitly requested. Current E0–E4 GMF/rollback results cannot substitute for those tests.
