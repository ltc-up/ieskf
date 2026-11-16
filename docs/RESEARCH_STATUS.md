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

## Phase AM — real NDT integrity diagnostics, PARTIAL

All284 frozen Sources assessed;35 new CUDA fits over5 fixed seven-seed groups. Two groups have competing converged low-fitness peaks (up to3.467m displacement), retained AMBIGUOUS without independent truth. Rank/support/normal diagnostics are not covariance;155 converged single-starts remain UNKNOWN. Warm batches22–28ms,cold223ms; no end-to-end20Hz qualification.6 new tests pass. [Report](../experiments/phase_am_ndt_integrity/FINAL_CONCLUSION.md).

## Phase AN — executable global management; qualified real fusion NO-GO

Same284-frame window plus146 raw GNSS messages processed. Two AM ambiguities retained; map/antenna/time/fix-code/statistical GNSS prerequisites remain unqualified,so0 anchors/0 trusted availability.118 tests pass; synthetic recovery and isolated ROS2 transport(3654 events) pass,not a real recovery/accuracy qualification. Local wheel dZ4.177m and sigmaZ51.783m persist. Strict controlled A/B/C and production/20Hz admission not established. [Report](../experiments/phase_an_global_multisource/FINAL_CONCLUSION.md) · [cross-stage summary](../experiments/PHASE_AL_AN_FINAL_SUMMARY.md).

## Phase AO — normal full-lap qualification NO-GO

Long real-input attempts now supersede any inference of local stability from14.7s: loop8 produces245568 outputs over2455.68s but Z span310.906km and wheel dZ597.146m; loop44 also fails vertical behavior. These are model-output failures, not independent accuracy errors. Primary loop35 wheel timestamps are zero, loop7 has an early IMU gap. No qualified complete causal NDT/GNSS lap, map/GNSS independent truth, trusted global coverage or real-time ROS equivalence. New bounded local checkpoint replay exactly matches Header order on43441 real events; production six-state lag/smoothing and actual callback timing remain distinct. Preserve architecture isolation and full local P; qualify local physical/time/model contracts and raw candidate provenance before production admission. AP is diagnosis-only under AO NO-GO. [AO](../experiments/phase_ao_full_lap_validation/FINAL_CONCLUSION.md).

## Phase AP — qualification boundary preserved, robustness acceptance NO-GO

No AO source/config/threshold changes. Whole-record global injection tests prove no local writeback but0 trusted coverage is a localization failure,not robustness success. Map support ratio responds to a small injected hole while rank/classification miss it;existing historical multimodes lack true/wrong labels. GNSS and partial-LiDAR recovery remain unqualified;no20Hz/production admission.134 cumulative tests pass. First restore qualified local sensor/time/reference/vertical behavior and full candidate provenance,then isolated causal-lap/independent-reference validation. [Cross-stage conclusion](../experiments/PHASE_AO_AP_FINAL_SUMMARY.md).

## AQ1 correction to AO/AP interpretation

AO/AP tested no-NDT motion recursion, not complete15D NDT loose coupling. AQ1 verifies all-zero original8 wheel channel despite nonzero Kistler; long unobserved/unqualified prior then amplifies small innovations through full cross covariance. No wheel/F sign bug found; no gain/P masking or weight inflation. Restore real external pose constraints in a short causal normal test; keep physical/runtime calibration and covariance qualification explicit. [AQ1](../experiments/phase_aq1_motion_model_audit/FINAL_CONCLUSION.md).

## AQ2 — complete short loose-coupled dataflow restored

Real raw120 clouds drive branch-own15D histories, CUDA Deskew, current seeds and delayed map updates; C30/40 uses actual Z. Persistent private adapter fixes cold process lifecycle without production/Q/R changes; earlier failures retained. Normal20s request stops at2.70s after55 evaluated frames, so no normal-lap qualification. Original8 all-zero wheel is a protocol/input-eligibility failure per new user provenance; newer wheel research should use loop10, older data loop44. No accuracy or covariance calibration claim. [AQ2](../experiments/phase_aq2_ndt_15d_closed_loop/FINAL_CONCLUSION.md).

## AQ3 and final qualification

Native chart transport mathematics tested; no statistically calibrated NDT R or independent accuracy. AQ2 preserved real short XYZ/yaw closure30/40, normal extension stops55/401 at2.70s. AQ3 prior Native/multistart GPU logs lost with temporary environment; transcript summaries cannot replace retained artifacts.153 tests pass (19 added across AQ), but normal-lap/production NO-GO. Rebuild private adapter/re-extract and durably rerun before extending normal qualification. [Final summary](../experiments/PHASE_AQ_FINAL_SUMMARY.md).

## AR1 — real20s continuous pose feedback restored

Target freshly exported from map0105 equals AQ2 SHA. Original35iteration55-frame failure reproduced; corrected derivative alone unchanged.100budget XYZ/yaw stops75frames with drifting pitch. Existing fullSO3 map pose observation plus100budget completes401/40120s with unchanged engineering R/Q; no production modification or low-fitness promotion. Frozen extension candidate, not independent accuracy or whole-lap admission.

## AR2 — loop44/map0105 actual normal lap completed

Same frozen full-pose100iteration candidate runs all1468 lap triplets through real CUDA Deskew/NDT and historical full15D updates,100%accepted; max accepted gap51.004ms, no LOST.20s401/401 and60s1201/1201;40-frame native-reference rerun exact. Wheel maxZcorrection3.32mm on lap, Zspan10.324m follows similar production height shape without proof of true accuracy. Independent reference/engineering R statistics and ROS20Hz tails remain unqualified; recovery not exercised. Functional and declared normal research availability GO, production NO-GO. Earlier XYZ/yaw and derivative-only failures retained.

## AS1 — engineering-reference accuracy and multilap study

75ms historicalGPS1 engineering-reference comparison givesAR15D/EKF XYRMSE.577/1.138m on81.5%lap,not intrinsic superiority:−75ms sensitivity reversesranking;right-curvaturesegments3/9 worsen lateralP95.3continuouslaps4134/4134updates,common2mbin lateralrangeP95.994m versusold.837m includes realdriving. Thirdlapreference absent. Native1468/1468butpairedreferenceXYRMSE.628vsFitness.573,lateralP95.679vs.493: no advantage established. Geometry/rank andfitness do not certifypeak;4007VALID-diagnostics/127DEGRADED,heldoutaccuracy unavailable. No actualbody/tyre safetyGO or productionadmission. [ASreport](../experiments/phase_as_accuracy_quality/FINAL_CONCLUSION.md).

## AS2 — CPU/GC long-tail reproduced without estimator changes

Instrumented850framecausalprefixmatchesARseeds/finalposes/full15Ddeltasexactly. Worst368.05msframecontains321.59msgen2GCwithin325msNDTingest;originalAR360mshadnotGCtrace,sooriginalcauseconditional.4/1468ARderivedcomputeintervals>50ms,profile1/850fullframes>50ms. RecordedmodelcompleteomitslaterCPU/logwork;notphysicalROSpublishtime. Targetbuild_msisretainedstartupmetadata. Needboundedmemory/logging/actualarrivalpublicationaudit;noalgorithm/GCdisabletuning. ROSonlineandproductionnotqualified. [FinalASqualifications](../experiments/phase_as_accuracy_quality/FINAL_CONCLUSION.md).

## Phase AT

GNSS reconstruction implementation equivalence VERIFIED; actual75ms latency, missing recorded antenna TF/phase-centre calibration and physical map height reference remain conditional/unvalidated. Documented origin-height difference accounts for28.46m but leaves about−.561m mean Z difference; not a certified geoid correction. Full eight-vertex historical visual-box footprint checks execute on1468 new/old poses and1197 valid GNSS poses without modeled crossing; real tyre/physical body and surveyed boundary safety remain NOT_VALIDATED. Right3/9 native-time prior/NDT/posterior lateral P95 .721/.286/.674 and .686/.437/.668m: larger discrepancy exists before current NDT update. Physical source and correct-peak label UNKNOWN. No estimator changes, GNSS fusion, tuned delay/lever/R or production qualification. 171 tests pass. See [AT report](../experiments/phase_at_gnss_footprint_corners/FINAL_CONCLUSION.md).

## AU1 — Full state history recovered without changing AR

Instrumented recorded-NDT envelope replay and canonical Header replay reproduce all1468 native priors exactly; envelope also exactly matches complete published means/P/times, gains and15D deltas. Private full pre/post traces retained:23535 propagation,7357wheel,1468NDT canonical calls. Repropagation revisions are separate; no duplicated observations are accumulated. Physical sensor/reference/noise qualification remains conditional, and original CPU pre-ingest sub-boundary is unavailable. 174 tests pass. [Audit](../experiments/phase_au_prediction_feedback/PREDICTION_MODEL_AUDIT.md).

AU2: about10% full unit lateral gain is explained by current prior variance versus engineering R, with small pure-direction coupling difference; covariance calibration remains absent. Entry right3/9 discrepancies are mostly inherited, and moving-axis transport cannot be ignored. New relative motion can oppose updates in3, but9 has small net new NDT-referenced discrepancy; GNSS-reference trends differ. No Q/R or physical model change is justified by these diagnostics alone. [Balance](../experiments/phase_au_prediction_feedback/PREDICTION_CORRECTION_BALANCE.md).

AU3: lawful fixed-state P/R scaling shows gain alone cannot separate P versus R calibration; all cross terms retained.250ms-cutoff fixed-observation skips retain prior NDT influence into the next prior in selected right3/9 examples, refuting simple overwriting there. Actual matcher feedback under changed seeds is NOT_VALIDATED. Full signed pose-variance partition and correct postreset uncertainty projections retained.183 tests pass,13 frozen used-input/transaction hashes match; original model/AR/AS/AT source unchanged. Physical mechanism remains conditional; no Q/R/model modification warranted. [AU conclusion](../experiments/phase_au_prediction_feedback/FINAL_CONCLUSION.md).

## Phase AY candidate implementation

Additional raw optical lateral/forward motion information is implemented without changing original IMU/wheel/NDT equations or arbitrary LiDAR gain changes. Full cross-covariance/Joseph/reset and bounded delayed replay retained; optical installation, temporal correlation and engineering velocity variance are not statistically certified. The optional16D installation state is an experiment, not calibrated vehicle configuration. See [AY design](../experiments/phase_ay_localization_solution/ALGORITHM_AND_EVIDENCE.md).

AY final: actual motion information produces conditional difficult-corner/global improvement without changing LiDAR gain rules, but fails no-normal-regression (segment8 lateralP95 .343576→.436548m). Static mounting-yaw augmentation does not cure it. All three candidates complete1468+4134 own-state CUDA/Deskew loops;195 numerical tests pass, no production/default changes. Full offline cycle profiling P95/P99/max23.095/26.069/76.524ms; hard50ms deadline not met, ROS2/100Hz/independent accuracy unvalidated.150ms reference reverses global ordering. Preserve original baseline; candidate is not main/default qualified. [Final AY](../experiments/phase_ay_localization_solution/FINAL_CONCLUSION.md).


## Phase BD — P0 in progress

Independent AY-based branch. Verified distinct step/objective epsilons and step-min coupling;13×1468 frozen CUDA fits, selected MT and frozen AY1468/4134 causal runs retained.75ms conditional gains,150ms ranking reversal and normal5 regression; no production qualification or accepted rejection policy. Remaining BD queue counterfactual/health/search/recovery work open. [P0 report](../experiments/phase_bd_ndt_health_bridge/P0_CLOSED_LOOP_COMPARISON.md).


### BD result: safe rejection policy NOT ESTABLISHED

P0 conditional75ms gains with150ms reversal and normal5 regression; true CUDA1468/4134 retained. Independent KD-tree2×2 and exact-radius oracle completed; NO-GO replacingDIRECT7 (normal8/history basin regressions). Original15D delayed counterfactual baseline exactly reproduces;30-frame rejection cases quantify drift/reentry, full prediction-only fails. HEALTHY necessary checks do not certify peaks; no new final gate admitted. Automatic global recovery NO-GO. [Final conclusion](../experiments/phase_bd_ndt_health_bridge/FINAL_CONCLUSION.md).


BD publication: committed locally on the new-volume independent branch; Git HTTPS authentication failed. No remote-access claim, force push or main modification. See PUBLICATION_STATUS.md in the phase directory.


## Phase BE design status

BE is documentation-only on an independent research branch, based on05b73c0. Literature and official source snapshots support a staged plan: observation lifecycle/reference-point audit, coupled directional diagnostics and lateral-velocity audit, matched-input NDT/small_gicp Shadow, then a two-branch independent-history Gaussian mixture if evidence permits. No new algorithm, Q/R, TF or production default was changed; no BE accuracy or50ms qualification is claimed. GNSS remains solely under existing production use_gps semantics, default-off for planned experiments; no RAIM/FDE/integrity state machine. Default legacy GPS1 converter input has no active antenna-to-rear-axle correction; the separate TF-capable VectorNav path is not evidence that this EKF input is corrected. Runtime wiring remains unverified. [Recommendation](../experiments/phase_be_multimodal_localization/FINAL_RECOMMENDATION.md), [architecture](../experiments/phase_be_multimodal_localization/MULTIMODAL_LOCALIZATION_ARCHITECTURE.md), [GNSS audit](../experiments/phase_be_multimodal_localization/GNSS_INTEGRITY_AND_FUSION.md).


## BE-E0–E4 result: prototype implemented, system qualification FAILED

218 tests and actual single-/three-lap own-state replay attempts completed. Directional charts and bounded full-P rollback have explicit numerical contracts; real direction selection never drops an axis, so degeneracy benefit is unproven. NDT/GICP candidates differ but strict75ms proxy labels show no good/bad rescue. GMF single lap does not switch/improve; both GMF three-lap attempts are incomplete after severe latency/expiry and a retained forward-horizon guard. OpticalXY conditional gain still regresses normal8 and reverses at150ms. Production six-state queue audit is separate; no production accuracy/timing claim. Currentaa50 lidar_ndt source usesGLL3, not the oldc647 OFF assumption; deployed component remains unverified. GNSS stays existing-switch-only; legacy antenna reference-point defect not fixed. Main untouched. BE5 is queued for read-only A/C/covariance review before any approved code change. [Evidence](../experiments/phase_be_e0_e4/FINAL_CONCLUSION.md).


## BE5 Stage 1: review only; implementation pending

Storage prerequisite completed under the approved duplicate-cloud plan, with original inputs and full numerical/failed-run evidence retained privately. BE5 reviews actual15D input/state/delay contract, vertical proposal versus commit, A fixed-world versus body-normal models, C correlated/unknown attitude sources, and Native full covariance charts/scales. No trustworthy independent C or absolute6D Native calibration is established. Recommend small opt-in frozen diagnostics and Native-helper decoupling only after explicit approval; no B, new matchers, optimizer/Q/Bias/production changes. [Review](../experiments/phase_be5_stage1/README.md).


## BE5 Stage 2A complete — no permission to fuse or replay

303 numerical tests pass; original Legacy proposals reproduced exactly in49 stored BE priors. Native full covariance no longer requires Fitness to pass Legacy range, but no scale calibration or wrong-peak guarantee. C reference remains unavailable; oldZ/12632 missing-contract comparisons are explicitly blocked. A2 produces one beyond-domain accel-bias proposal. Recommend no immediate A soft fusion; Stage2A has0 commits to live/carried state/P,0 new matches and0 replays. Production/Q/Bias/gates and oldBE reports stay unchanged. [Evidence](../experiments/phase_be5_stage2a/README.md).


## Phase BF / BF0 — public data approval checkpoint

Baseline da3ea9df, metadata/disk/paper audit only. KITTI0002:77syncedscans and846extractOXTSrecords;3archives814833599bytes; proposed8GiBnew-volumepeak, nocleanupneeded. FutureRACECARS3Mscene19.3GiB, separateapprovalrequired. OfficialRAWIMUexampleaccelerationassignmentbug andSiPSPDF403 retainedaslimitations. CalibrationZIP4068byteswasreadinmemoryduringmetadataquery(preapprovalboundarydeviation), notsaved/extracted. NoBFmatching/replay/algorithmchange. AwaitKITTIapprovalandstop. [Checkpoint](../experiments/phase_bf_public_dataset_validation/README.md).


## Phase BF1 — KITTI0002 functional validation, no production qualification

Only approved3archives,77LiDAR/846OXTS; actualtimestampinterpolationfix keepsmap0–19/test20–76.57frames completeeach,46validupdates/11MAX_ITERATIONS. SamepriorfullR/K/Pcandidates andcontrolledarrivalB0/B1 retained. NativeXYreference .220625→.196802m butlateralP95/Yawregress;50msbudgetfails.47targetedtestspass, noA/Cfusion orproduction/Q/Bias/NDTchange. Sharedreference/map/receiverlimits ruleoutindependentcalibration. RACECARnotdownloaded;separate19.289GiBsource/64GiBnew-volumeplanawaitsapproval. [Results](../experiments/phase_bf_public_dataset_validation/BF1_KITTI_REPORT.md).


## BF1只读复盘完成

末11帧支持下降、纵向弱约束及优化偏移并存，种子并未先偏数米；64/65帧已有收敛但变差。缺逐迭代证据，震荡机制UNKNOWN。Native XY收益以纵向和拒绝期状态记忆为主，横向/Yaw代价保留；NDT约78%墙钟、纯IO/IMU细分未知，50ms仍NO-GO。仅分析原记录，零下载/匹配/更新/回放。RACECAR用户不批准，历史申请归档；下一步5帧trace/计时只提案，停止等待单独授权。[报告](../experiments/phase_bf_public_dataset_validation/review/README.md)。


## BF1-P2 five-frame diagnostic complete

Exactly5fixedregistrations60/64/65/66/76,119iterations;historicallibrary/outputparityverified. Allscoresincrease,0GLLbacktracks,noobserveddirectionreversal.64/65convergetoworse-referencepositions;66stepfails;76last10stepscappedwhilemovingfarther. Fitnesswall+sync35.47–36.53msdominatesalign,notGLLretries. AssociationidentityandfineGPUtimingUNKNOWN.5nonCUDAcontracttestspass;onefloat-analysisfailurepreserved/correctedwithoutnewmatches. No15Dreplay/update,parameterorproductionchange. Stop;nextFitness/optimizercontrastsneedseparateapproval.[Report](../experiments/phase_bf_public_dataset_validation/p2/README.md).


## BF1-P3 bounded Fitness ON/OFF complete

Frames60/65/76 only,3repeats/mode=18serialmatches,qualifiedoriginalsharedlibrary. OFFFitnessexplicitnull, noLegacyR. Match/score/Nativefullcovariance/exitandqualityequalhistoricalP2(numericdifference0).Steadypairedalignsavings36.197/36.547/35.691ms;postreturnOFFsync<=0.001593ms,no35mswaitingtransferseen. PCLwallincludessync;pureGPUeventsplitUNKNOWN.6no-CUDAtestspass.No15Dreplay/production/R/Q/Bias/optimizerchange. Not20Hzqualification. Stop;newFitnessoptimizationnotauthorized.[Report](../experiments/phase_bf_public_dataset_validation/p3/README.md).


## BF1-P4 bounded source ablation complete

Fiveframes,72newmatches(20A+2FULL+40B+10C),P3FitnessOFF,unchangedTarget/Seed/optimizer.60spatialcasesconverge,maxsameNseedspread0.0823m;FULLandrandom10kframe76stillfail.Normal60regresses;pointcountnotuniversalquality.20kspatialcoversall1mSourcecells,radial/heightweightsdiffergreatlyfromFULL.Pythonsampling73–76msmakesonline50msNO-GO.FirsttriggeromittedFULL,preserved/correctedbeforeOXTSread;BcompletedafterCwithoriginalseeds.NoA2RLmatch/filter/productionchange.Stop.[Reports](../experiments/phase_bf_public_dataset_validation/p4/README.md).

## Phase BG — prior-map FAR-LIO review only

No new localization result. Official nine-state backend/internal tilt and prior-PCD path distinguished from the research15D. Existing upstream no-deskew fusion loses point timestamps; direct remapping is blocked. Native workspace dependency/storage approval and verified fusion/IMU/map/init contracts precede any finite run. No container pulled, no algorithm changed, no full replay. [Report](../experiments/phase_bg_far_lio/FINAL_RECOMMENDATION.md).

## Phase BG — Native finite tests, system NO-GO

Original-map FAR-LIO and research-only timestamp/lever adapters exercised on preregistered normal8/right3. Local raw-registration benefit does not establish improved delivered localization. IO/clock stalls, dropped scored outputs and ~10m converged right-turn candidates prevent qualification; native callback speed does not certify full50ms. Preserve failures and clarify data-path timing before any separately approved follow-up. Production6D equivalence, vertical accuracy and independent wrong-basin identification remain UNKNOWN. [Evidence](../experiments/phase_bg_far_lio/native/README.md).


## BG-C1 pit连续启动预检

恢复轮速/评分消息时间线；共同地图初始化成功证据不足，按停止门未运行FAR/15D。没有新精度或实时性结论。见[时间线](../experiments/phase_bg_far_lio/c1/LOOP44_STARTUP_TIMELINE.md)及[结论](../experiments/phase_bg_far_lio/c1/FINAL_CONCLUSION.md)。


## BG-C1追加授权：录制均值初始化连续对照完成

最新授权解除历史init_flag阻塞。两套各一次pit到345.806s，完整输入一致；新FAR右弯状态XY约0.716m、新15D约0.890m，旧10m候选未复现。初始化与调度共同改变，不能归因单一匹配器。FAR全段148ms尾延迟及7个缺失输出，严格50ms仍NO-GO。见[完整交付](../experiments/phase_bg_far_lio/c1/recorded_init/README.md)。旧停止报告只是前置历史，不代表本轮未运行。


## BG-C2只读更新：先审信息预算，再谈减维

FAR九状态名义body惯性与15D map惯性形式可等价，当前输入、Bias、倾角、Wheel和P策略不同。源码确认FAR低速学习gyroBias、驾驶中冻结，未输入Wheel的非完整零vy分支不执行；ROS received缓存会每tick重新提交，核心清fusion_vec不足以防重复。C1机制勘误已附，原结果保留。

评分段FAR Rx/Ry恒.05，无位置/姿态裁剪证据；15D保存增益横向投影中位约.103，单次信息接入仍沿用×5 Legacy。不能直接除5或宣称R就是根因。15D回滚P95 3.224ms/max5.204ms，无500ms固定输出延迟；C1 Legacy不支持retract，不能冒称完整终态恢复。Bias数值稳定不等于物理可观测，FAR Jacobian还存在源码线性化缺项。优先观测ID/信息预算/增益证据，再独立标定方向性R，模型减维只作待批方案；生产替换仍NO-GO。[审查](../experiments/phase_bg_far_lio/c2/README.md)。


## BG-C3：统一降R暂不进入闭环

Legacy×5的即时影响已量化，但不能由固定prior推断闭环轨迹贡献；normal8与Yaw/错误候选代价保留，单一增强信息NO-GO。保留15D/Wheel/回滚，优先独立100Hz只读快照合同及历史错峰证据补齐；几何/时序信任设计尚无验证，不部署在线自适应R或9D。[报告](../experiments/phase_bg_far_lio/c3/README.md)。


## BG-C4：高频架构、FAR部分归因与1013错峰，PARTIAL

35次固定输入CUDA诊断复现收敛错峰最大约4.506m；纠正旧397s与bag713s属于不同经过，713组18例11收敛/7迭代失败，未复现多米错峰。三地图控制含补结构后仍4.306m的反例；错峰局部投影仍正定。FAR已有日志恢复356个受限prior（right3 104tick），其prior/post横向P95约.531/.523m；原生同50ms一次/五次信息消融证实重复缓存压P，但真实全部贡献分解仍UNKNOWN。原生预测有限差分支持速度-倾角A缺项，未修改。事件内核+只读100Hz实际ROS合成401/401slot、mean/P逐事件等价；16接口测试及18合同检查通过，停输STALE保留。没有新真实1013长时15D闭环或生产50ms/100Hz资格；保留惯性/Wheel/完整P/回滚，不复制FAR重复信息，也不部署新单指标门控。下一步仅不改轨迹的真实发布资格与观测事务/独立分支证据。见[BG-C4完整报告](../experiments/phase_bg_far_lio/c4/README.md)。
