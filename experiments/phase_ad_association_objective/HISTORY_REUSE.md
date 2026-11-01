# History reuse and execution budget

Executed AD: seven saved AC cycle segments (two original, five nearest-valid), 33 initial path samples each, 43 detected boundary records refined by 40 bisections, 181 smooth-cell derivative comparisons; one unified objective and nine NEW synthetic tests; 13 SAME deterministic solver starts; uniform endpoint precision probes; curvature/reset/atomic diagnostic at the nominal endpoint; no continuous replay.

Historical A/B: directly reuse all 13 original and 13 nearest-valid AC outcomes, not re-run their optimizations. AC's five-cycle and two-cycle states are fixed payloads. Keep their source/map/prior/config binding, including original shared inertial preprocessing and native LiDAR reference time. No U19/V34/X45/Y equivalence/KKT, W iteration-limit, AB gate proof or old continuous replay is repeated.

The numerical reference is the exact same full15 frozen prediction/P. Alternative seeds are optimizer starts only; no recentering of the prior or choosing a favorable seed. No new real-data sample was selected. AD uses no reference trajectory for optimization or evaluation. No independent accuracy qualification exists.

Raw experiment count: 13 new dynamic-objective solves, 13 uniform endpoint precision probes, 1 discarded local-energy covariance transaction. New filter covariance commits=0; continuous runs=0. The precision probes reuse endpoints instead of re-running the 13 capture trajectories. All raw failures remain recorded.


Evidence boundary: executed frozen-input results are stated above; AC results are explicitly historical reuse. Physical pose correctness, calibrated uncertainty, and continuous Z/vz improvement are not inferred. Raw points, full states/P, map, timestamps, identities and sensitive configuration stay in ignored local data. Speculative next steps are not executed results.
