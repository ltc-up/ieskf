# Research status

A complete inertial error-state research prototype exists. Pose/full-state elimination and fixed-association constrained optimization were investigated numerically; those findings are not a deployment qualification.

Dynamic map association remains unqualified for reliable convergence. A first non-converged geometry update can over-contract position uncertainty while leaving velocity uncertainty substantial; propagation then creates strong position–velocity conditional recovery. Large proposed other-state corrections are mathematically explained, but their physical validity is not established.

AA preserved the prediction, wheel and geometry models and investigated actual carried state separately from raw proposals. Authorized de-identified aggregate results are published in the self-contained Phase AA report; full event-level results remain private. Typical geometry improvements do not imply elimination of worst cases; rejecting a proposal is not the same as observing a real state spike.

Next priority: update convergence/association eligibility and credible information/posterior handling, while maintaining a fixed independent preprocessing front end. No independent localization accuracy, long-term statistical calibration, live scheduling or production suitability has been established. The production system remains unchanged.

Research recording policy: each completed or failed phase must include self-contained results/failures, manifest input/config/code identity, index/status updates, reviewed public commit and a normal push to main. Failed authentication or remote conflicts must be reported; never force-push. See AGENTS.md.

## Latest evidence: Phase AB

Final checks distinguish end-of-budget convergence from partial decrease: 26/226 old iteration-limit acceptances pass a conservative local/pair certificate. The first accepted partial update's Z variance contraction is numerically reproduced but not statistically justified; exact Target normalization works, broader correlation remains unresolved. A qualification-only atomic gate passed eight new tests and five frozen implementation checks, yet its one continuous run rejected all 284 LiDAR frames and exactly matched the inertial/wheel-only control. This is a failed operational candidate, not smoother/better localization.

Do not extend replay or loosen gates to rescue acceptance. Next priority is a certified first-frame/local capture solution with consistent association and credible information commitment; preserve prediction/wheel/noise and legal initialization contracts. Fixed-factor solvability alone is insufficient. Independent accuracy, long-term uncertainty calibration, live scheduling and production readiness remain unvalidated.

## Latest evidence: Phase AC

A first-frame failure mechanism is now directly isolated: nearest raw Target selection followed by plane-validity rejection creates an alternating factor deletion despite accurate fixed-factor solves and inactive first-frame safety guards. Sixty outer steps and twelve prescribed starts repeat the same two-cycle. One nearest-admissible selection candidate removes that local hole, but twelve starts (including nominal) converge only to a five-cycle involving remaining radius/plane switches. One other prescribed start gives an interior endpoint; this is not a replacement initializer, not the original-selector solution, and not reliable capture.

No AC continuous replay was admitted. Conditional posterior/state transaction works numerically at the isolated endpoint but remains statistically uncalibrated; Z-vz improvement is unvalidated. Next focus: association/support selection and an optimization/measurement merit model that remain consistent under re-selection, rather than larger budgets, favorable seeds, relaxed safety or covariance/noise scaling. Preserve failures and input/prior identity. Longer replay and production qualification remain unavailable.

## Current update: Phase AD

Direct one-sided measurements establish finite selected-merit jumps in AC cycles, including normal validity, radius, valid-plane loss and repeat-weight changes. A single explicit compact Huber plane-mixture objective and full derivative eliminate observed two/five cycles in13 frozen starts. Raw solver7/13 stationary; six precision stalls remain explicit, with uniform tiny Newton probes certifying all13 endpoints. Nominal root now exists under this NEW energy; there are two local minima, not known correct poses. No seed promotion/no noise, motion, prior, data or safety retuning.

A numerical local-energy Laplace transaction succeeds, but is NOT a validated recursively justified LiDAR posterior. It makes sigmaZ even tighter (~0.00783m); strong association-kernel forces and density sensitivity require an observation/information model check. No AD continuous replay, actual Z-vz improvement or independent accuracy. Raw median4.723s fails20Hz. Next investigate that measurement objective/information meaning and integrate robust numerical certification before longer replay; do not merely add iterations or loosen safeguards. [Phase AD](../experiments/phase_ad_association_objective/FINAL_CONCLUSION.md).

## Current update: Phase AE — measurement semantics block filter integration

AD energy/code unchanged; no Q/R/P, motion, Deskew, map or engineering-limit change. Exact Target aliases change latent null/plane weights, gradients and optima, not linearly independent observation count. Fixed-plane half/dense sampling changes nominal pose and13-start attraction; normal refit adds separate neighborhood effects. Exact full15 signed Hessian chain rule reconstructs numerical curvature; intrinsic plane weights are themselves density-dependent, so algebraic fractions are not independent physical information.

Decisive synthetic counterexample: an infinite exact plane has zero X/Y/yaw information but sampled-grid energy shrinks sigmaX/Y1m→0.0745m and yaw5°→0.6505°, with halfspace-density-induced5.44cm drift. Hence0.00783m Z curvature is not calibrated evidence. Four NEW train-only spatial folds share no normal-neighbor IDs with validation, but low coverage/metric conflicts and no absolute reference prevent accuracy ranking; historical G1/G2 posthoc scores are contaminated. All50 diagnostics and failures retained;10 new tests PASS.

Current decision:FILTER_POSTERIOR_NOT_QUALIFIED; no AE continuous/closed-loop run or production qualification. Preserve smooth association tools as research, prioritize defensible surface/sampling/plane uncertainty and separate frontend proposals from backend information. Do not repair by covariance scaling, noise tuning or velocity clipping. [Phase AE](../experiments/phase_ae_measurement_information/FINAL_CONCLUSION.md).

## Latest evidence: Phase AF

The candidate frontend and surface backend are now explicitly separated. Known physical surfaces yield correct rank3/5/6, exact Target/Source alias invariance and finite shared-error confidence floors.23 new tests validate full covariance/latent marginalization/Joseph/reset and atomic rollback. Splitting one real plane into independent patches remains an information-counting counterexample.

Real deterministic grouping is only a hypothesis; fits and conditional information vary with tiling and the transferred synthetic Sigma is not map calibration. A frozen linear proposal fails the unchanged trust guard; no real state/P commits or continuous runs. G1/G2 cannot be uniquely validated from shared candidate-generation data. Next priority: independently establish physical plane identity, shared map/Source error and reference quality, then admissible first-frame capture. AD Hessian must not become filter information. Numerical posterior validity does not establish statistical or production qualification. [Phase AF](../experiments/phase_af_surface_observation/FINAL_CONCLUSION.md).

## Latest evidence: Phase AG

Original first-frame prediction is not proven outside every feasible conditional capture path. With AF5m hypothesized surfaces and unchanged synthetic uncertainty, frozen-prior feasible steps converge after9 outer/27 inner steps; final dZ1.036m and pose/bias norms satisfy original limits. Exact final factor retention plus12 tiny probes and inherited optimality pass. This is not an independently correct pose or calibrated observation.2.5m and10m diagnostics fail different qualification conditions, confirming model/grouping dependence.

Data audit found9 different map files, but no verified independent acquisition/build membership/datum contract or first-frame qualified reference. Real plane/common-map/Source/prior correlations remain uncalibrated.0 real state/P commits and no continuous replay. Next priority is independent physical surface/error/reference evidence; preserve conditional capture as a diagnostic, do not promote synthetic Sigma or relax guards. Short closed-loop admission remains FAIL. [Phase AG](../experiments/phase_ag_surface_evidence_capture/FINAL_CONCLUSION.md).

## Latest evidence: Phase AH

Five AA frozen events isolate actual base-pose kernel Z translation and RP rotation. A bitwise reproduces original Source/old update; same-input full6D NDT remains paired-controlled. Z translation ablation moves matched Z3.7–42.4mm, RP ablation−18.1–+9.5mm, not proportional to raw far-point changes. Source effects on full15 proposals are real, but all three inputs still partially accept first MAX_ITERATIONS and shrink sigmaZ1.077m to about0.016/0.017/0.025m. All next-frame large proposals reject.0 strict certificates in15 frozen updates; no new continuous result.

B was corrected to the requested kernel-base operator; earlier IMU-origin B excluded. FixedXY/yaw score-only diagnostics remain unqualified and cannot partition direct-vs-coupled height changes. Evidence favors reliable geometry/information commitment before altering Deskew or velocity/Q/R; reject-all is still not a successful locator. No independent height/accuracy/statistical/production qualification. [Phase AH](../experiments/phase_ah_deskew_z_ablation/FINAL_CONCLUSION.md).

## Latest evidence: Phase AI

Five existing NDT poses pass pose guards, but conditional full15 completion fails bias guards in three. Original four-loop strict qualification is 0/5 for both starts. One limited 16-loop diagnostic qualifies E5/NDT at loop 7; the first frame still fails from both starts. Its certified E5 conditional posterior is numerically valid, not statistically calibrated. No state/P commits or replay. NDT initialization is not a first-frame solution; stop expanding seeds/budgets and preserve the distinction between optimization and information credibility.

## Latest evidence: Phase AJ

Direct NDT XY/yaw is now an independent loose-observation comparison on the original15D predictor. Eleven focused tests and153 saved-prior checks validate the observation/Joseph/reset/atomic interface. A single causal14.70s replay commits66/284, rejects218 and has a5.50s LiDAR gap; no point-plane factor is fused. Direct LiDAR |dvz| peaks at.161m/s, yet unchanged-wheel full-P updates produce4.137m Z/.560m/s vz corrections. Final sigma Z47.596m documents missing vertical observability. The interface is useful as a negative engineering baseline, not stable3D localization, a repair of geometry convergence, calibrated noise, or20Hz production proof. Do not extend unchanged replay or tune R/guards to rescue this outcome; prioritize legitimate height/feedback evidence.

AJ identifies a direct rejection mechanism:28/87 guard-rejected proposals have planar step <=.5m but unsafe full3D steps. At L079 a converged NDT with small XY innovation induces dZ−.551m through full P and loses the whole observation. This supports prioritizing vertical observability/feedback evidence over blaming only NDT convergence, or clipping hidden-state increments.

## Current recommendation: Phase AK

Prefer NDT/GNSS global Map/Odom correction over direct unreliable map-to-local15D feedback. Preserve local full covariance and physical velocity models; local wheel-induced vertical corrections remain unresolved. FAR-LIO audited at ef20510316ad1f8dbf2e428e4a193f700493eccd:5/9-state alternatives, handler gyro bias, source status/timeout and source-diagonal covariance adaptation are distinct from15D ESKF. Borrow source management design, not whole LIO or unvalidated zero-velocity model assumptions.

7 new synthetic tests and94 existing tests pass; Schmidt cross covariance/PSD and projection information loss verified. Synthetic global errors8/11.18m coexist with exact protected local parity; GNSS position recovery does not recover yaw. Rank3/5/6 and equal-cost distinct Gaussian-cell peaks separate degeneracy from aliasing. No calibrated global covariance, real peak rate, independent accuracy or ROS closed-loop qualification. First next step is additive NDT candidate/health export and shadow global correction, with explicit UNKNOWN uncertainty and feature-flag rollback. [AK](../experiments/phase_ak_fusion_architecture/FINAL_CONCLUSION.md).

## Phase AL — shadow implementation, PARTIAL

Real284-frame input replay:155 shadow candidates/129 nonconvergences,0 trusted anchors; all1470 local state/P outputs equal no-map control bitwise. Time/version/epoch/interpolation/duplicate contracts implemented; ROS2 isolated real-input transport runs. Frozen AJ seeds/Deskew are retained and correlated, not an autonomous new frontend. No calibrated global covariance or accuracy. [Report](../experiments/phase_al_shadow_map_odom/FINAL_CONCLUSION.md).
