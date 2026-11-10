"""Rebuild public aggregate reports/plots without exposing private trajectories."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).parent
load=lambda p:json.loads((D/p).read_text())
C=load('gicp_lap_metrics.json');B=load('native_historical_metrics.json');F=load('ndt_fixed_lap_metrics.json');M=load('gicp_multilap_metrics.json');AB=load('matcher_ab_metrics.json');FB=load('gicp_multilap_feedback.json');P=load('profile_full_lap_metrics.json')
cs=C['sensitivity']['0.075']['historical_common_mask'];bs=B['sensitivity']['0.075']['historical_common_mask'];fs=F['sensitivity']['0.075']['historical_common_mask']
write=lambda name,s:(D/(name+'.md')).write_text(s.strip()+'\n')
def row(method,metrics):return '|'+method+'|'+'|'.join(f"{metrics[k][s]:.3f}" for k,s in [('XY','rmse'),('abs_longitudinal','p50'),('abs_longitudinal','p95'),('abs_longitudinal','max'),('abs_lateral','p50'),('abs_lateral','p95'),('abs_lateral','max'),('yaw_reference_abs_deg','p95')])+'|\n'
table='|Group|XY RMSE m|Long P50|P95|max m|Lat P50|P95|max m|Yaw engineering P95 deg|\n|---|---:|---:|---:|---:|---:|---:|---:|---:|\n'
for name,metrics in [('A NDT Fitness',cs['baseline']),('B Native historical',bs['candidate']),('C GICP fixedR',cs['candidate']),('sameR NDT control',fs['candidate']),('E historical oldEKF',cs['old_EKF'])]:table+=row(name,metrics)
corner='|Fixed segment|A NDT Fitness lateralP95|B Native|C GICP fixedR|NDT sameR|E oldEKF m|\n|---|---:|---:|---:|---:|---:|\n'
for c in cs['corners']:
 k=c['id'];b=next(x for x in bs['corners'] if x['id']==k);f=next(x for x in fs['corners'] if x['id']==k);corner+=f'|{k}|'+ '|'.join(f"{v['abs_lateral']['p95']:.3f}" for v in [c['baseline'],b['candidate'],c['candidate'],f['candidate'],c['old_EKF']])+'|\n'
time='|Reference assignment|A NDT Fitness|B Native|C GICP|NDT sameR|E oldEKF XY RMSE m|\n|---|---:|---:|---:|---:|---:|\n'
for d in ['0.0','0.075','0.15']:
 c=C['sensitivity'][d]['historical_common_mask'];b=B['sensitivity'][d]['historical_common_mask'];f=F['sensitivity'][d]['historical_common_mask'];time+=f'|{float(d)*1000:.0f}ms|'+ '|'.join(f"{v['XY']['rmse']:.3f}" for v in [c['baseline'],b['candidate'],c['candidate'],f['candidate'],c['old_EKF']])+'|\n'
write('ACCURACY_COMPARISON',f'''# Same native-time engineering comparison; no certified truth

All primary single-lap groups use the fixed historical common1197/1468 sample mask (81.54%), fixed AS tangent/segments, map/base rear reference and AT full-attitude antenna conversion. No GNSS enters filters. OldEKF is a recorded engineering comparator. Physical antenna installation,75ms delay, map datum and independent INS attitude are not certified; all accuracy differences are CONDITIONAL.

{table}

{corner}

GICP's posterior worsens both difficult3/9 and all fixed normal-segment lateralP95 relative to originalA. It also loses primary globalXY. Against the **same fixedR** NDT control, GICP's XY improves0.612→0.596m, but not uniformly across directions/corners; it remains worse than originalFitnessA. Therefore the R difference confounds a naive backend verdict. Do not declare the matcher intrinsically worse solely from A/C, or better from one corner/raw pose.

{time}

Every preset hypothesis is retained, none selected for promotion. Historical Native's previously reported0.628 versus0.573 values used another evaluation sampling convention; current0.631/0.577 are consistent native-time/common-mask comparisons, not a redesignedNative method. Z differs about−29m because datum/reference remains unqualified; Yaw/velocity/height are engineering differences/diagnostics, not certified accuracies.

D (new adaptive fusion) is NOT_IMPLEMENTED because AW1/AW2 do not justify a statistical direction-to-noise mapping. AV's mixed improvements are not a default or an additional successful method. Full individual-corner P50/P95/max/longitudinal/Yaw distributions, update fractions and all time assumptions are in the aggregate JSON files; failed/nonaccepted frames are retained, not erased from coverage.

Conditional historical visual-box clearance: GICP minimum0.548m, NDT original0.440m, oldEKF0.139m;0 modeled containment failures in each single-lap test. This is not actual tyre/vehicle safety certification and cannot outweigh worse reference error. Private full track/box/corner plots remain in ignored evaluation directories.
''')
raw=AB['reference_sensitivity']['0.075'];rt='|Common eligible raw matches|NDT|GICP|\n|---|---:|---:|\n'
for field in ['XY','abs_longitudinal','abs_lateral']:rt+=f'|{field} RMSE m|{raw["NDT"][field]["rmse"]:.3f}|{raw["GICP"][field]["rmse"]:.3f}|\n'
write('MATCHER_AB_COMPARISON',f'''# 1468 same-Source/same-float-seed actual matcher transactions

Every frame uses frozen AR Deskew Source SHA and the same historical float seed; neither backend supplies feedback in this **diagnosticAB**. Both matchers actually run. CUDA results reproduce original1468 final transforms exactly (max0). small_gicp uses its actual CPU solver, not stored NDT poses. Closed-loop evidence is separately reported.

NDT1468/1468 valid+converged; GICP1399/1468 (95.30%). These are optimizer/format qualifications, not correct-peak labels. Native zero-based iteration index is preserved. Both target paths bind the same map0105 source Target; local representations/neighborhoods differ as documented.

{rt}

Raw-reference comparison above uses **1141 common eligible historical-reference samples**. NDT own-valid reference count1197 is also retained (`own_valid_XY`); restricting NDT to GICP-success frames must not be mistaken for its full output coverage. Whole-lap raw same-input XY is nearly equal under75ms. Matching and filtered-posterior errors are separate metrics, not interchangeable.

GICP raw right3/9 lateralP95≈0.247/0.402m vs NDT≈0.290/0.438m on the common mask; segment1 improves, segment5 worsens≈0.429→0.862m. Relative final position/body direction/Yaw distributions and six preselected repeat/probe cases are in matcher_ab_metrics.json. No raw-cost or rawH magnitude is compared across objectives.

Fixed midpoint probes cover0/1/3/5/8/9, ±1.5m lateral/±.1deg yaw. GICP local solutions differ up to0.319m in5 and0.152m in3; other tested cases are closer. Same-initialization repeats agree numerically. These are local-basin/association sensitivity observations, not independent correct/wrong labels. Zero score or converged flag with zero pairs is invalid; the initial adapter failure demonstrates this distinction.

Final-factor cost perturbations: maximum absolute diagonal curvature discrepancy from GN across the36 tested axes is≈0.00170 relative. This supports a local conditional metric near these selected solutions, not rematching/global covariance. Full factorH, optimizer-lastH, gradient, cost, inliers and timing remain private.
''')
laps='|Preselected lap|Frames|GICP updates|GNSS raw valid|GICP XY RMSE|original NDT XY RMSE m|\n|---|---:|---:|---:|---:|---:|\n'
# Acceptance per lap is recovered once from existing private transactions by archival manifest.
counts=[1468,1352,1314]
lap_updates=load('manifest.json')['actual_runs']['gicp_multilap']['preselected_lap_updates']
for r,n,updates in zip(M['per_preselected_lap_75ms_raw_reference'],counts,lap_updates):laps+=f"|{r['lap']}|{n}|{updates}|{r['valid']}|{r['candidate']['XY']['rmse']:.3f}|{r['baseline']['XY']['rmse']:.3f}|\n"
write('CLOSED_LOOP_RESULTS',f'''# Actual own-state GICP, complete lap and complete continuous three laps

A2ndt-free GICP loop actually reads original three raw LiDAR inputs, own IMU/wheel historical trajectory, persistent CUDA Deskew, own Source, own seed/ROI, CPU small_gicp, fixed6D engineeringR, complete15D update and suffix repropagation. Only initial state follows the same lawful frozen initialization; no subsequent production/other-branch state or future GNSS is fed back. Original state/P cross terms, SO3, lever and500ms cache remain.

|Actual run|Requested/processed|Updates|Rejected matching candidates|Longest update loss|
|---|---:|---:|---:|---:|
|GICP lap|1468/1468|1401|67|0.25003s|
|GICP continuous3laps|4134/4134|3956|178|0.25003s|
|NDT same fixedR control|1468/1468|1468|0|normal50ms cadence|

No per-lap reset, slowed input clock, extended retention, discarded slow frame or frozen NDT measurement injection. Actual matching backend is CPU small_gicp; CUDA is used for Deskew. Legacy`ndt` transaction key/rejection string are compatibility labels; annotated metadata fixes the inherited runner's misleading CUDA/Fitness descriptor while preserving raw summaries.

{laps}

Per-lap engineering-reference results worsen relative to originalNDT, most strongly in third lap. Different1204 raw-mask versus1197 historical-mask first-lap RMSE values must not be mixed. Extended raw reference uses the same frozen AT math and explicit masks, not a newly fitted truth.

All saved P are symmetric and numericallypositive; complete15D deltas equal fullK×innovation. StateZ remains bounded over the observed map-height span; this is not a certified height or Bias accuracy claim. Source/matcher results change with own feedback; full private state/P/innovations/parameters/failed matches are retained. No wrong-peak correctness/detection accuracy is established by healthy numeric recursion.

Fresh legacy40 exactly reproduces AR. A separate fully timed1468-frame GICP replay agrees with the mainGICP lap to≈1.14e−13 pose,8.89e−16P and identical acceptance; this validates instrumentation/memory-management equivalence. The repeat is not a distinct held-out accuracy trial.
''')
perf='|Component, actual continuous3lap samples|P50|P95|P99|max ms|\n|---|---:|---:|---:|---:|\n'
for key in ['gpu_deskew_ms','crop_voxel_sample_ms','rebase_with_file_write_ms','source_tree_ms','source_covariance_ms','align_wall_ms','information_refresh_ms','service_wall_ms','model_observation_age_ms']:
 v=FB['timing'][key];perf+='|'+key+'|'+'|'.join(f'{v[k]:.3f}' for k in ['p50','p95','p99','max'])+'|\n'
prof='|Inclusive timed full-lap operation|Calls|P50|P95|P99|max ms|\n|---|---:|---:|---:|---:|---:|\n'
for key in ['imu_predict','wheel_correct','external_pose_correct','ingest_ndt']:
 v=P['component_timings'][key];prof+=f"|{key}|{v['n']}|"+'|'.join(f'{v[k]:.3f}' for k in ['p50','p95','p99','max'])+'|\n'
write('PERFORMANCE',f'''# Offline throughput measured; strict deadline/ROS qualification absent

Measured warm-start startup: GICP lap{C['performance']['startup_ms']:.3f}ms,3lap{M['performance']['startup_ms']:.3f}ms, including original Deskew context and static CPU target preparation; separate from per-frame processing.

Same-input AB align P95: CUDA-NDT2.820ms, small_gicp6.950ms. Continuous own-state GICP alignP958.357ms; prior loop7's18.79ms cannot be compared as the same workload. Current Source is only a few thousand voxel points and Target/preprocessing differ from those older experiments. No claim of CUDA GICP, GPU acceleration of GICP or algorithm score comparability.

{perf}

GICP4134 processing-section wall P50/P95/P99/max17.740/26.966/37.187/109.653ms. Nominal50ms budget is usually met in thisoffline processing section, but strict full-run50ms deadline FAILS because of the tail. Max modeled Header-to-completion287.331ms;0 above500ms history. Raw arrival delay and actual computation/queue are different quantities. Model clock uses recorded availability plus actual elapsed processing; it is not a wall-clock ROS executor. Source input order and history horizon are unchanged.

{prof}

`ingest_ndt` includes external update and suffix replay, so it MUST NOT be added to `external_pose_correct` P95. IMU/wheel calls include replay repeats; call count is not distinct input count. Module percentiles cannot be summed into end-to-endP95. Whole processing-section timing excludes some final JSON materialization at end-of-run, and modeled completion can omit some serialization after its mark; no physical20Hz freshness/100Hz publisher guarantee follows.

Fully profiled actual1468 replay resource record:48.14s userCPU,3.17s systemCPU,30.34s elapsed,169% jobCPU,610020KiB time-reported maximumRSS. This is the reported process-family maximum, not summed simultaneous processes, device memory or portable CPU utilization. TBB arena4 is frozen. Static global target prep/tree/covariance happens once; per-frame Source prep, finalH refresh and own-state replay remain included. CUDA Deskew persists; GPU per-process peak/utilization was NOT_LOGGED and is not fabricated.

Runtime memory management streams JSONL, retains bounded Python records and freezes static input GC generation while GC stays enabled; it does not disableGC, fake completion or extend history. Legacy40 and complete profiling equivalence are verified. Actual4134 completion is a meaningful improvement over AV's growing-ledger runtime failures, not proof all-load real-time behavior. No controlled AV-V3 rerun with this memory change was performed; algorithm costs also differ, so current completion cannot prove GC/ledger retention was the unique cause or cure of AV failures. ROS20Hz/100Hz NOT_VALIDATED; production NO-GO. CPU GICP is not currently expensive enough, nor accurate enough, to justify a CUDA port as the priority.
''')
# Only de-identified aggregate plots, never absolute track/vehicle geometry.
x=np.arange(len(cs['corners']));fig,ax=plt.subplots(figsize=(11,4))
for j,(label,vals) in enumerate([('NDT Fitness',[r['baseline']['abs_lateral']['p95'] for r in cs['corners']]),('Native',[r['candidate']['abs_lateral']['p95'] for r in bs['corners']]),('GICP',[r['candidate']['abs_lateral']['p95'] for r in cs['corners']])]):ax.bar(x+(j-1)*.25,vals,.25,label=label)
ax.set_xticks(x,[r['id'] for r in cs['corners']]);ax.set_xlabel('fixed AS segment');ax.set_ylabel('75ms engineering lateral P95 m');ax.legend();fig.tight_layout();fig.savefig(D/'corner_comparison.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4))
for label,r,key in [('NDT Fitness',C,'baseline'),('Native',B,'candidate'),('GICP',C,'candidate')]:ax.plot([0,75,150],[r['sensitivity'][d]['historical_common_mask'][key]['XY']['rmse'] for d in ['0.0','0.075','0.15']],'-o',label=label)
ax.set_xlabel('predefined engineering reference assignment ms');ax.set_ylabel('XY RMSE m');ax.legend();fig.tight_layout();fig.savefig(D/'reference_sensitivity.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4))
for ax,k in zip(axes,['3','9']):
 d=AB['geometry_by_fixed_corner'][k];ax.bar(np.arange(6)-.18,d['ndt_conditional_energy_mean'],.36,label='Native shape');ax.bar(np.arange(6)+.18,d['gicp_conditional_energy_mean'],.36,label='GICP shape');ax.set_xticks(range(6),['long','lat','Z','roll','pitch','yaw']);ax.set_title('fixed corner'+k);ax.set_ylabel('normalized conditional energy');ax.legend()
fig.tight_layout();fig.savefig(D/'directional_geometry.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4));ax.bar(['P50','P95','P99','max'],[17.7395905,26.9659446,37.18748802,109.652611]);ax.axhline(50,color='r',ls='--',label='50ms computation budget');ax.set_ylabel('actual3lap processing section ms');ax.legend();fig.tight_layout();fig.savefig(D/'processing_quantiles.png',dpi=150);plt.close(fig)
