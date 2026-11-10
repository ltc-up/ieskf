"""Generate public aggregate reports/figures; never load/publish a trajectory."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--private-root',type=Path);a=p.parse_args();root=Path(__file__).parent
if a.private_root:
 for name in ['A_lap_metrics','A_multilap_metrics','B_lap_metrics','diagnostics','ndt_numeric','ndt_density','raw_moment_validation','legacy_reproduction','B_failure_probe']:
  r=json.loads((a.private_root/(name+'.json')).read_text());(root/(name+'.json')).write_text(json.dumps(r,indent=2)+'\n')
 # Run summaries contain no coordinates, Header values or private paths.
 r={}
 for name in ['A40_v1','A40_v2','A40_v3','A40_v4','A_lap','A_multilap','A_instrumented_lap','B40','B40_rawplanes','B_lap','legacy40']:
  s=json.loads((a.private_root/(name+'_summary.json')).read_text());r[name]={k:s[k] for k in ['requested_frames','processed_frames','delayed_15D_updates','duration_s','stop_reason','decisions','wall_ms']};r[name]['semantic_note']='direct updates are conditional MAP; legacy matching_converged/component labels do not certify per-direction observability' if name!='legacy40' else 'actual original CUDA-NDT pose filter'
 (root/'executed_runs.json').write_text(json.dumps(r,indent=2)+'\n')
load=lambda n:json.loads((root/(n+'.json')).read_text());A=load('A_lap_metrics');M=load('A_multilap_metrics');B=load('B_lap_metrics');D=load('diagnostics');runs=load('executed_runs');AW=json.loads((root.parent/'phase_aw_native_gicp_fusion/gicp_lap_metrics.json').read_text());aw=AW['sensitivity']['0.075']['historical_common_mask'];s=A['sensitivity']['0.075']['historical_common'];awcorn={str(c['id']):c for c in aw['corners']};fmt=lambda x:f'{x:.4f}'
rows=['|Fixed segment|Original NDT pose lateralP95 m|AW GICP pose lateralP95 m|AX-A lateralP95 m|','|---|---:|---:|---:|']
for c,v in s['corners'].items():rows.append(f"|{c}|{fmt(v['baseline_NDT_pose']['lateral']['p95'])}|{fmt(awcorn[c]['candidate']['abs_lateral']['p95'])}|{fmt(v['AX_direct']['lateral']['p95'])}|")
report='''# Controlled accuracy comparison — engineering reference only

All methods use frozen loop44/map0105 and fixed native-time AT conversion. A/GICP-pose comparison reuses AW actual own-state runs; AX-A/B use new own-state direct residual runs. No GNSS is fused and no reference time/Mask/extrinsic is fitted. Current historical common1197 samples cover81.54% of the full lap. Report all processed frames and all rejections separately; reference coverage is not localization coverage.

|Full1468 lap,75ms convention|XY RMSE m|Qualification|
|---|---:|---|
|Original CUDA-NDT pose+ESKF|0.5771|historical actual baseline, CONDITIONAL reference|
|AW small_gicp pose+ESKF|0.5957|historical actual baseline, CONDITIONAL reference|
|AX-A GICP normal direct-IESKF|0.7778|actual full loop; replacement FAILED|
|Old EKF|1.1375|engineering comparison, not truth|
|AX-B native raw-moment surface|NOT_A_FULL_LAP|531/1468; cannot use its prefix RMSE as a full-lap score|

'''+ '\n'.join(rows)+'''

The two difficult corners and normal5/8 all worsen in AX-A. Three continuous laps do not rescue the claim. Under raw-reference masks, AX-A/original XY RMSE by lap:

|Lap|Reference samples|AX-A m|Original m|
|---|---:|---:|---:|
'''
for i,r in enumerate(M['per_lap_75ms_raw'],1):report+=f"|{i}|{r['valid']}|{fmt(r['AX_direct']['XY']['rmse'])}|{fmt(r['baseline_NDT_pose']['XY']['rmse'])}|\n"
report+='''
All0/75/150ms overall XY/longitudinal/lateral/Yaw/Z and fixed segment metrics remain in A_lap_metrics.json/A_multilap_metrics.json. Z differences are unqualified datum differences, not height accuracy. Time sensitivity does not authorize choosing0ms when AX appears closer than the original. AW Native historicalB is0.631m on the same historical convention and is not a direct residual candidate.

AX-B has only531 processed frames,451 updates and326 common-reference samples; its failed-prefix75msXY RMSE16.275m is not comparable to a complete-lap score. Difficult3/9 occur outside its achieved prefix: NOT_EXECUTED, not improved. Its complete531-frame state history remains private with complete rejected tail.
'''
(root/'ACCURACY_COMPARISON.md').write_text(report)
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
ks=list(s['corners']);x=np.arange(len(ks));fig,ax=plt.subplots(figsize=(10,4));
for j,(label,key) in enumerate([('Original NDT','baseline_NDT_pose'),('AX-A direct','AX_direct')]):ax.bar(x+(j-.5)*.25,[s['corners'][k][key]['lateral']['p95'] for k in ks],.25,label=label)
ax.set_xticks(x,ks);ax.set_xlabel('fixed segment');ax.set_ylabel('75ms engineering lateral P95 m');ax.legend();fig.tight_layout();fig.savefig(root/'corner_aggregate.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(6,4));
for k,label in [('AX_direct','AX-A'),('baseline_NDT_pose','Original NDT'),('old_EKF','Old EKF')]:ax.plot([0,75,150],[A['sensitivity'][d]['historical_common']['overall'][k]['XY']['rmse'] for d in ['0.0','0.075','0.15']],'-o',label=label)
ax.set_xlabel('preset reference delay ms');ax.set_ylabel('engineering XY RMSE m');ax.legend();fig.tight_layout();fig.savefig(root/'reference_sensitivity.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(6,4));
for k,label in [('AX_direct','AX-A'),('baseline_NDT_pose','Original NDT')]:ax.plot([1,2,3],[r[k]['XY']['rmse'] for r in M['per_lap_75ms_raw']],'-o',label=label)
ax.set_xlabel('preselected continuous lap');ax.set_ylabel('raw-reference XY RMSE m');ax.legend();fig.tight_layout();fig.savefig(root/'multilap_aggregate.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4));tags=['A lap','A3lap','B failed prefix'];keys=['A_lap','A_multilap','B_lap'];x=np.arange(3)
for j,k in enumerate(['p50','p95','p99','max']):ax.bar(x+(j-1.5)*.18,[runs[n]['wall_ms'][k] for n in keys],.18,label=k)
ax.axhline(50,color='r',ls='--');ax.set_xticks(x,tags);ax.set_ylabel('actual offline processing section ms');ax.legend();fig.tight_layout();fig.savefig(root/'processing_aggregate.png');plt.close(fig)
