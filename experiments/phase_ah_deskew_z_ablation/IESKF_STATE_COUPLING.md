# Frozen15D update sensitivity

Historical U0 source audit distinguishes the production6-state planar EKF (XY/yaw/yaw-bias/forward-speed/yaw-rate, separate1D Z/Roll/Pitch filters) from this15D research state. These prototype vZ corrections must not be relabeled as production EKF vZ outputs.

All cases use the original AA per-event x-/P-, raw IMU/wheel history, old four-iteration Huber/target-normalization, original matching/engineering guards. No AF/AG plane segmentation/noise, no Q/R/P tuning, no velocity cap. A matches original AA accept/reject labels and P to<2.69e-11. NDT poses are NOT fused; direct point-plane geometry consumes each ablated Source under the same original predictor/update.

Three meanings remain separate:

- PROPOSED_CORRECTION: last untruncated absolute frozen-chart MAP proposal; earlier legal scaled steps may differ.
- LEGACY_FROZEN_ACCEPTED: final old research transaction x/P returned in this isolated test. MAX_ITERATIONS partial acceptance is known unqualified; it is not proof of convergence. Baseline A corresponds to the historical carried AA event, but B/C are NOT continuous trajectories.
- STRICT_SUBMITTED/PUBLISHED_STATE: all15 fail inherited strict local+association certification; no state/P is carried to another event or published. A blanket reject-all is not claimed as an improved locator.

|Event|Group|Raw proposed dZ m|Raw proposed dvZ m/s|Legacy frozen accepted dvZ m/s|Old exit|Legacy sigmaZ m|
|---|---|---:|---:|---:|---|---:|
|E1|A|0.936400|0.652306|0.652306|MAX_ITERATIONS|0.015979|
|E1|B|0.928065|0.644208|0.644208|MAX_ITERATIONS|0.017258|
|E1|C|0.811056|0.559419|0.559419|MAX_ITERATIONS|0.025455|
|E2|A|0.250287|4.809311|0.000000|LINE_SEARCH_OR_TRUST_LIMIT|0.094415|
|E2|B|0.265960|5.116874|0.000000|LINE_SEARCH_OR_TRUST_LIMIT|0.094415|
|E2|C|0.252789|4.862431|0.000000|LINE_SEARCH_OR_TRUST_LIMIT|0.094415|
|E3|A|0.213748|1.455888|1.081766|MAX_ITERATIONS|0.014371|
|E3|B|0.248789|1.686153|1.211468|MAX_ITERATIONS|0.017183|
|E3|C|0.209724|1.430105|1.073985|MAX_ITERATIONS|0.013718|

E1 starts sigmaZ1.077427m. ALL A/B/C old exits are MAX_ITERATIONS, yet conditional sigmaZ collapses to0.015979/0.017258/0.025455m. This is a common strong, unqualified posterior contraction, not evidence of calibrated true uncertainty. Removing either compensation does not remove it. E2 uses the SAME A-history prior in every group: prior rho(Z,vZ)=0.985528, and raw proposals4.809/5.117/4.862m/s are ALL rejected by old guards. Their legacy accepted dvZ is0; they are not propagated velocity spikes. E3 old partial accepts1.082/1.211/1.074m/s in isolation; only A is an already observed AA carried event. No autonomous B/C feedback claim.

The full local conditional recovery is `deltaOther=solve(P_pose,P_pose,other).T deltaPose` with pose indices[0,1,2,6,7,8], other[v,bg,ba]. No covariance crosses are deleted. Signed contribution table for E2's last raw dvZ:

|Group|X|Y|Z|Roll chart|Pitch chart|Yaw chart|Sum m/s|
|---|---:|---:|---:|---:|---:|---:|---:|
|A|+0.021463|+0.088555|+4.864960|-0.008829|-0.152840|-0.003997|+4.809311|
|B|+0.023090|+0.093144|+5.169600|-0.013746|-0.151163|-0.004050|+5.116874|
|C|+0.021603|+0.088363|+4.913597|-0.010097|-0.147021|-0.004013|+4.862431|

All signed sums match complete proposals at numerical precision; terms can cancel. These are derivatives in declared map-position/right-body-attitude coordinates, not six independent sensor innovations. P(vz,Z)/P(Z,Z) alone is NOT the six-variable conditional coefficient. All events' complete contributions, proposed/legacy accepted Roll/Pitch/Bias summaries, P_ZZ/P_vzvz/P_vzZ/rho before/after and strict-retained-prior values are in key_results.json; full P/K stay private.

The first event has smaller normalized cross amplification than E2/E3; after the historical first tight-Z update and propagation, small pose requests can imply much larger velocity proposals. The direct source ablation changes the request, while frozen P fixes the recovery operator. E2 B changes matched NDT Z by+34.94mm yet changes raw geometry dvZ by+0.3076m/s; NDT Z is not itself the geometry innovation, so this is a paired sensitivity observation, not a scalar transfer function. The real backend amplification mechanism survives all three inputs. Its long-term magnitude and physical correctness cannot be inferred without independent truth and a valid continuous observer.
