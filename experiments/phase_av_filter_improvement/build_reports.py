"""Build de-identified reports/plots from committed aggregate metrics only."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
D=Path(__file__).parent
load=lambda n:json.loads((D/n).read_text())
v=load('v3_lap_metrics.json');b=load('single_use_metrics.json');m=load('v3_multilap_metrics.json');feedback=load('v3_lap_feedback.json')['fixed_AS_corners'];main=v['sensitivity']['0.075']['historical_common_mask'];rows=main['corners']
def write(name,s):(D/(name+'.md')).write_text(s.strip()+'\n')
corner_table='|Fixed segment|Legacy P50/P95/max m|V3 P50/P95/max m|Old EKF P50/P95/max m|\n|---|---|---|---|\n'
for c in rows:
 corner_table+='|'+str(c['id'])+'|'+'|'.join('/'.join(f"{c[k]['abs_lateral'][q]:.3f}" for q in ['p50','p95','max']) for k in ['baseline','candidate','old_EKF'])+'|\n'
write('CORNER_COMPARISON',f'''# Actual single-lap fixed-corner comparison

All values are native-time **GNSS engineering-reference differences**, using the frozen75ms hypothesis and historical common mask1197/1468 (81.54%). Same AS/AT segment definitions/tangents; no recentering, alignment fitting, time selection or GNSS feedback. OldEKF is recorded historical output, not an independently rerun/qualified truth.

{corner_table}

Right3 P95 improves26.6%, right9 improves17.3%, control1 improves27.5%. But segments0/5/6/7/8 worsen (including small changes); segment5 rises0.414→0.460m and8 rises0.344→0.387m. The predeclared no-other-corner-degradation test therefore FAILS, without post hoc tolerance relaxation. OldEKF remains better than V3 in the difficult right3/9 under this reference.

V3 full unit lateral effectiveK mean at right3/9 is{feedback['3']['gain']['mean']:.4f}/{feedback['9']['gain']['mean']:.4f}, versus AU original0.1032/0.1055. Mean native lateral innovation becomes{feedback['3']['innovation']['mean']:.4f}/{feedback['9']['innovation']['mean']:.4f}m, compared with original−0.152/−0.119m. Actual mean rear-reference lateral correction is{feedback['3']['actual_update']['mean']:.4f}/{feedback['9']['actual_update']['mean']:.4f}m. Smaller corrections after trajectory changes do not imply lower gain: residual history is already different. Full15D and attitude-lever terms are retained. The nonlinear mixture derivative is logged separately from effectiveK.

Restoring packet information does not multiply steady-state gain by5: repeated posterior shrinkage also lowers subsequent P. Right3/9 lateral prior standard-deviation diagnostics are{feedback['3']['prior_lateral_std']['mean']:.4f}/{feedback['9']['prior_lateral_std']['mean']:.4f}m. This reinforces that packet bookkeeping is not proof of calibrated P/R or a complete physical repair.

Private native-overlay/corner1/3/9 figures and full state histories are retained; public figures contain aggregate bars only. Adjacent frames are correlated, so no independent-sample significance or detection accuracy is asserted.
''')
sensitivity='|GNSS assignment hypothesis|Legacy XY RMSE m|V3 XY RMSE m|Old EKF XY RMSE m|\n|---|---:|---:|---:|\n'
for delay in ['0.0','0.075','0.15']:
 r=v['sensitivity'][delay]['historical_common_mask'];sensitivity+=f"|{float(delay)*1000:.0f}ms|"+'|'.join(f"{r[k]['XY']['rmse']:.3f}" for k in ['baseline','candidate','old_EKF'])+'|\n'
laps='|Preselected lap|Processed frames|Raw-reference valid|V3 XY RMSE m|Legacy XY RMSE m|Old EKF XY RMSE m|\n|---|---:|---:|---:|---:|---:|\n'
for r,n in zip(m['per_preselected_lap_75ms_raw_reference'],[1468,1352,638]):laps+=f"|{r['lap']}{' prefix' if r['lap']==3 else ''}|{n}|{r['valid']}|"+'|'.join(f"{r[k]['XY']['rmse']:.3f}" for k in ['candidate','baseline','old_EKF'])+'|\n'
write('ACCURACY_AND_ROBUSTNESS',f'''# Mixed actual feedback outcome; full repair FAILED

Primary frozen75ms historical common mask: V3 XY RMSE0.590507m versus legacy0.577097m (+2.3%); B0 single-use alone0.594154m. Difficult right3/9 improve, but normal regions/globalXY degrade. Passing arithmetic, raising gain, or completing all matches cannot override the frozen screen. No full-fix/default promotion follows.

{sensitivity}

All three hypotheses are retained; they are reference assumptions, not calibrated physical delays. GNSS is never fused, fitted, used to tune covariance/prior or select the best offset. AT's full-attitude antenna-to-rear conversion is reused. Actual antenna installation, physical delay, map datum/altitude and independent attitude/truth remain unqualified. Z reference differs about−29m; this is not certified height error. Yaw/longitudinal/lateral/Z difference distributions and coverage are in JSON, not promoted to certified accuracy.

The extended raw GPS1 reference uses the same frozen AT conversion/protocol mask beyond the historical220s CSV. It is separately labelled `raw_extended_reference`; historical1197-mask metrics and raw1204-mask first-lap metrics must not be mixed. The latter explains0.592 versus0.591, without fitting a delay. It permits conditional evaluation of later preselected laps, not creation of certified truth.

{laps}

This is one continuous candidate run, no per-lap reinitialization. Third lap contains only638/1314 frames;3458/4134 overall (83.65%), interrupted at172.850s by cache/backlog. Lap2/third-prefix use unselected-in-time data but strongly correlated same-track/sensor inputs, not independent statistical calibration. Candidate globalXY worsens in both: no robust out-of-sample superiority. No completed three-lap claim.

Single-lap center matches1468/1468 converge, all accepted;55 use rank4 common combinations and1413 full rank6. Continuous prefix3458/3458 converge/accept,113 partial. These are conditional optimization/observation events, not known-correct-peak counts. Bounded probes can discover local competing basins; common wrong peaks, incomplete hypothesis sets and selection uncertainty remain possible. Wrong-peak precision/recall, missing-map robustness and calibrated NIS/NEES remain NOT_VALIDATED.

V3 lap P minimum eigenvalue5.56e−7, prefix5.01e−7; all saved P symmetric. Lap Z span10.311m, vz−2.151..2.140m/s; prefix Z span10.340m, vz−2.179..2.268m/s. No freezingZ/cross removal/Bias clipping. These bounded diagnostics do not establish height/velocity accuracy. Lap maximum wheelZ correction3.321mm, NDTZ correction0.513m, NDT velocity-change norm0.398m/s. Full Bias/state aggregates are retained; no numeric divergence observed before scheduling failure.

Conditional historical visual-box minimum clearance: candidate0.399m, legacy0.440m, oldEKF0.139m,0 modeled non-contained frames. Candidate clearance worsens relative to legacy. Historical visualization geometry is not certified car/tyre outline; reference/boundary survey error bounds absent. Physical tyre safety and production safety remain NOT_VALIDATED/NO-GO.
''')
write('CUDA_CLOSED_LOOP',f'''# Real own-state CUDA feedback

Fresh direct legacy40 and wrapper legacy40 reproduce AR exactly: seed, prior fullP, final CUDA pose, K and15D delta all max difference0. This precedes changed-algorithm evaluation; no frozen pose sequence was injected as a measurement.

Every candidate frame uses original three-LiDAR raw clouds, authorized persistent Deskew, its own historical motion/lever, its own seed/ROI, original map0105 Target and original fixed100-iteration CUDA binary. New observed pose commits at original Header time and repropagates later IMU/wheel. Original Q/R wheel/config/vehicle, matcher derivatives and per-point optimizer are unchanged. GNSS never enters the loop. Initialization is the same original lawful historical initialization; afterward no oldEKF state or future branch result enters feedback.

|Branch|Actual frames /1468|Accepted|Outcome|
|---|---:|---:|---|
|B0 packet-only|1468|1468|Mixed accuracy; full repair failed|
|V1 all-or-nothing ambiguity + hardNIS|881|796|2s loss gate; failed|
|V2 consensus projection + hardNIS|1159|1112|2s loss gate; failed|
|V3 robust consensus|1468|1468|Single-lap function complete, full repair failed|

V3 versus legacy:1467/1468 Source SHA change; first Source remains equal. Final NDT translation difference median7.97mm, P95{v['NDT_vs_baseline_translation']['p95']:.5f}m, max{v['NDT_vs_baseline_translation']['max']:.5f}m. Changed Deskew/initial seeds therefore caused genuine rematching, not fixed-observation replay. Small median differences still matter for some basins; neither convergence nor closeness certifies the correct map peak.

Three hypotheses from one cloud share one correlation group and are fused as one projected central observation, never three independent measurements. On ambiguous frames W removes all tested candidate-difference combinations; partial constraint status is explicit. It does not prove the true pose is inside that candidate set.

V3 original500ms multilap run completed1480 accepted frames before history assertion. Explicit2.1s retry completed3458 accepted frames then failed again as processing backlog grew. First1468 native Source/seed/P/match/delta/postresetP are exactly identical across cache settings. All failed Source/Target identity, hypotheses, state/P, per-transaction JSONL and error logs remain in persistent ignored storage. The retry is not a completed4134-frame experiment.
''')
write('PERFORMANCE',f'''# Offline timing and failed deadline qualification

Actual V3 single-lap startup465.787ms includes Target/GPU and CPU map-support preparation. Persistent contexts/Target are reused; no per-frame rebuild. Reported complete processing-section wall P50/P95/P99/max29.435/38.312/41.472/419.738ms. Original AR section P95 was19.24ms; extra hypotheses/geometry/robust likelihood cost is real, not free. Do not compare module P95 sums as transaction P95.

Center CUDA alignment P50/P95/P99/max = {v['performance']['center_align_ms']['p50']:.3f}/{v['performance']['center_align_ms']['p95']:.3f}/{v['performance']['center_align_ms']['p99']:.3f}/{v['performance']['center_align_ms']['max']:.3f}ms. Two extra matches, CPU support, IPC,128-node mixture and delayed suffix work add overhead. Mixture-alone/individual CPU scheduler/GC timings were not instrumented in this run, so precise bottleneck attribution is NOT_VALIDATED. AS GC findings are not proof of these exact pauses.

Interrupted3458-frame model Header-to-completion P50/P95/P99/max89.082/151.077/1032.714/2128.105ms. Center CUDA P954.069ms/P995.616ms/max7.178ms. Increasing cache retention500→2100ms permitted more processing but did not stop backlog growth. Completed native updates at50ms intervals are not fresh online localization when observed data arrive/finish seconds late. Health labels `TRACKING`/`PARTIAL_CONSTRAINT` in JSON describe native observation freshness only; do not present them as deadline-valid current output.

Original runner loses in-memory final wall array on assertion. Only per-transaction committed clocks and center/proxy timing survive; the multilap archive explicitly marks full wall-summary NOT_SAVED. Posthoc recovery does not fabricate omitted timings. Estimated completion clock omits some final advance/serialization work; it is not physical ROS end-to-end age. No new process-affinity, GC-disable or accelerated-clock trick was used.

Offline20Hz full-run deadline **FAILED** (long tails/cache loss). ROS20Hz observation and100Hz publisher **NOT_VALIDATED**: no production TF/node or live executor was run. Research runner produces state snapshots perLiDAR frame, not a qualified100Hz publisher. More cache is not a deadline fix. Next runtime work is bounded/chunked sensor/log retention, profiling actual allocation/GC/reprop/IPC, bounded latest-frame scheduling with explicit skipped/stale status, then independent end-to-end replay. No automatic algorithm/default change follows.
''')
# Aggregate-only plots: no absolute coordinates, full trajectories or vehicle geometry.
ids=[c['id'] for c in rows];x=np.arange(len(ids));fig,ax=plt.subplots(figsize=(10,4))
for j,(name,label) in enumerate([('baseline','legacy15D'),('candidate','V3'),('old_EKF','oldEKF')]):ax.bar(x+(j-1)*.25,[c[name]['abs_lateral']['p95'] for c in rows],.25,label=label)
ax.set_xticks(x,ids);ax.set_xlabel('fixed AS segment');ax.set_ylabel('75ms engineering lateral P95 (m)');ax.legend();fig.tight_layout();fig.savefig(D/'corner_aggregate.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4))
for name,label in [('baseline','legacy15D'),('candidate','V3'),('old_EKF','oldEKF')]:ax.plot([0,75,150],[v['sensitivity'][d]['historical_common_mask'][name]['XY']['rmse'] for d in ['0.0','0.075','0.15']],'-o',label=label)
ax.set_xlabel('predefined reference assignment ms');ax.set_ylabel('engineering XY RMSE m');ax.legend();fig.tight_layout();fig.savefig(D/'reference_sensitivity.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4));ax.bar(['P50','P95','P99','max'],[29.434881,38.31178025,41.47244413,419.738003]);ax.axhline(50,color='r',ls='--',label='50ms computation budget only');ax.set_ylabel('single-lap processing section ms');ax.legend();fig.tight_layout();fig.savefig(D/'processing_quantiles.png',dpi=150);plt.close(fig)
