"""Publish aggregate quantities only; no native timestamps, trajectory or TF."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--private-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
def read(n):return json.loads((a.private_root/n).read_text())
def stats(v):return dict(zip(['p50','p95','p99','max'],map(float,np.percentile(v,[50,95,99,100]))))
runs={}
for n in ['L_lap','L_multilap','XY_lap','XY_multilap','C_lap','C_multilap']:
 d=read(n+'_metrics.json');s=read(n+'_summary.json');r={k:d[k] for k in ['processed_frames','duration_s','accepted','rejected','NDT_converged','P_min_eigenvalue','P_symmetry_max','Z_span_m','vz_range','bg_norm','ba_norm','max_gap_s','unit_lateral_gain','lateral_innovation','actual_body_lateral_update','strict_repair_screen','screen_pass','performance']}
 r['transactions_sha256']=s['transactions_sha256'];r['reference']={k:{'valid':v['historical_common_mask']['valid'],'coverage':v['historical_common_mask']['coverage'],**{c:v['historical_common_mask'][c] for c in ['candidate','baseline','old_EKF']}} for k,v in d['sensitivity'].items()};r['corners75']=d['sensitivity']['0.075']['historical_common_mask']['corners'];r['laps75']=d['per_preselected_lap_75ms_raw_reference'];runs[n]=r
# Exact original-mode replay and profiling replay parity, excluding wall clock.
def parity(a,b):
 out={}
 for label,fn in [('seed',lambda r:r['seed']),('P',lambda r:r['prior']['P']),('NDT',lambda r:r['ndt']['final']),('delta',lambda r:r['update']['delta']),('K',lambda r:r['update']['K'])]:out[label]=float(max(np.max(np.abs(np.asarray(fn(x))-np.asarray(fn(y)))) for x,y in zip(a,b)))
 out['accepted_equal']=all(x['update']['accepted']==y['update']['accepted'] for x,y in zip(a,b));out['frames']=len(a);return out
base=json.loads(Path('data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json').read_text());par=parity(read('baseline40/transactions.json'),base[:40]);profilepar=parity(read('XY_cycle_lap/transactions.json'),read('XY_lap/transactions.json'))
probe=read('XY_cycle_lap/runtime_probe.json');gc=[e for e in probe['gc_events'] if e['frame']>=0];frames=probe['frames'];slow=max(frames,key=lambda e:e['ms']);perf=dict(inclusive_modules_ms={k:stats(v) for k,v in probe['modules'].items()},processing_ms=stats([x['ms'] for x in frames]),over50ms=sum(x['ms']>50 for x in frames),frames=len(frames),max_frame_index=slow['index'],max_frame_ms=slow['ms'],GC_at_max_frame=[e for e in gc if e['frame']==slow['index']],largest_in_loop_gc_ms=max([e['ms'] for e in gc],default=0),rss_peak_mib=max(x['maxrss_kib'] for x in frames)/1024,profiling_parity=profilepar)
perf['full_cycle_ms']=stats([x['cycle_ms'] for x in frames]);perf['full_cycle_over50ms']=sum(x['cycle_ms']>50 for x in frames)
# Replayed sensor updates have multiple revisions: last revision per message only.
for n in runs:
 records={}
 for line in (a.private_root/n/'optical_updates.jsonl').read_text().splitlines():
  e=json.loads(line);records[e['message_id']]=e['update']
 from collections import Counter
 runs[n]['optical_unique']=dict(total=len(records),decisions=dict(Counter(e.get('reason','UNKNOWN') for e in records.values())))
mechanism=read('mechanism.json')
for v in mechanism.values():
 v.pop('calibration_yaw_deg',None);v.pop('calibration_sigma_last_deg',None)
 for q in v['segments'].values():q.pop('calibration_yaw_deg',None)
summary=dict(qualification='Engineering GNSS reference only; no certified truth or calibrated covariance',baseline_commit='c647ac3224adee775d30dc6f9553c9b83612c060',baseline40_parity=par,runs=runs,optical_audit=read('optical_audit.json'),mechanism=mechanism,profile=perf)
(a.output/'aggregate.json').write_text(json.dumps(summary,indent=2)+'\n')
plt.rcParams.update({'font.size':9})
fig,ax=plt.subplots(figsize=(9,4));x=np.arange(10);ref=runs['XY_lap']['corners75'];ax.plot(x,[q['baseline']['abs_lateral']['p95'] for q in ref],'ko-',label='Frozen 15D')
for n,label in [('L_lap','Optical Y + wheel'),('XY_lap','Optical XY'),('C_lap','XY + static mounting yaw')]:ax.plot(x,[q['candidate']['abs_lateral']['p95'] for q in runs[n]['corners75']],'o-',label=label)
ax.set(xlabel='Fixed AS segment',ylabel='Lateral reference P95 (m)',xticks=x,title='Conditional engineering reference, fixed 75 ms; no runtime GNSS');ax.legend();fig.tight_layout();fig.savefig(a.output/'corners.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4))
for n,label in [('L_lap','Y + wheel'),('XY_lap','XY'),('C_lap','XY + yaw')]:
 axes[0].plot([0,75,150],[runs[n]['reference'][k]['candidate']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label=label)
axes[0].plot([0,75,150],[runs['XY_lap']['reference'][k]['baseline']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'ko-',label='Frozen');axes[0].set(xlabel='Predeclared reference shift (ms)',ylabel='XY reference RMSE (m)');axes[0].legend()
for n,label in [('XY_multilap','XY'),('C_multilap','XY + yaw')]:axes[1].plot([q['lap'] for q in runs[n]['laps75']],[q['candidate']['XY']['rmse'] for q in runs[n]['laps75']],'o-',label=label)
axes[1].plot([q['lap'] for q in runs['XY_multilap']['laps75']],[q['baseline']['XY']['rmse'] for q in runs['XY_multilap']['laps75']],'ko-',label='Frozen');axes[1].set(xlabel='Continuous lap (no reset)',ylabel='XY reference RMSE (m)',xticks=[1,2,3]);axes[1].legend();fig.suptitle('Conditional references; second/third laps use frozen raw-reference extension');fig.tight_layout();fig.savefig(a.output/'reference_and_multilap.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4));vals=np.array([x['cycle_ms'] for x in frames]);ax.hist(vals,bins=60);ax.axvline(50,color='r',label='50 ms input period');ax.set(xlabel='Measured full offline cycle (ms)',ylabel='Frames',title='Actual profiling replay, 1468 frames');ax.legend();fig.tight_layout();fig.savefig(a.output/'latency.png',dpi=160);plt.close(fig)
print(json.dumps({'baseline_parity':par,'profile':perf},indent=2))
