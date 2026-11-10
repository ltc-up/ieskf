"""De-identified run, source, fault and timing aggregates only."""
import argparse,json,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--ay-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
def read(n):return json.loads((a.root/n).read_text())
def stats(v):
 v=np.asarray(v);return dict(n=len(v),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),p99=float(np.percentile(v,99)),max=float(np.max(v))) if len(v) else dict(n=0)
runs={}
for n in ['X_lap','M_lap','M_multilap','S_lap','S_multilap','S_fault_lap']:
 d=read(n+'_metrics.json');r={k:d[k] for k in ['processed_frames','expected_frames','accepted','rejected','duration_s','NDT_converged','P_min_eigenvalue','P_symmetry_max','unit_lateral_gain','lateral_innovation','actual_body_lateral_update','Z_span_m','vz_range','bg_norm','ba_norm','performance','max_gap_s','strict_repair_screen']};r['transactions_sha256']=read(n+'_summary.json')['transactions_sha256'];r['sensitivity']={k:{'valid':v['historical_common_mask']['valid'],'coverage':v['historical_common_mask']['coverage'],**{j:v['historical_common_mask'][j] for j in ['candidate','baseline','old_EKF']}} for k,v in d['sensitivity'].items()};r['corners75']=d['sensitivity']['0.075']['historical_common_mask']['corners'];r['laps75']=d['per_preselected_lap_75ms_raw_reference']
 f=a.root/n/'manager.jsonl'
 if f.exists():
  last={}
  for line in f.open():
   e=json.loads(line);last[(e['kind'],e['message_id'])]=e
  es=sorted(last.values(),key=lambda e:(e['header_ns'],e['kind']));switch=[];previous=None
  for e in es:
   if e['phase']!=previous:switch.append(e);previous=e['phase']
  updates=[np.array(e['update']['delta']) for e in es if e['update'].get('accepted') and 'delta' in e['update']];deltas=np.array(updates)
  r['source']=dict(unique_events=len(es),phases=dict(collections.Counter(e['phase'] for e in es)),decisions=dict(collections.Counter(e['kind']+':'+e['update'].get('reason','UNKNOWN') for e in es)),transitions=len(switch),velocity_update_norm=stats(np.linalg.norm(deltas[:,3:6],axis=1)),position_update_norm=stats(np.linalg.norm(deltas[:,:3],axis=1)),rotation_update_norm=stats(np.linalg.norm(deltas[:,6:9],axis=1)),gyro_bias_update_norm=stats(np.linalg.norm(deltas[:,9:12],axis=1)),accel_bias_update_norm=stats(np.linalg.norm(deltas[:,12:15],axis=1)))
  if n=='S_fault_lap':
   origin=min(e['header_ns'] for e in json.loads((a.root/'optical.json').read_text())['events'])+1_000_000_000
   fault=[]
   for low,high in [(20,22),(35,35.5)]:
    inside=[e for e in es if low<=(e['header_ns']-origin)*1e-9<high];after=[e for e in es if (e['header_ns']-origin)*1e-9>=high and e['kind']=='optical' and e['update'].get('accepted')];fault.append(dict(interval_s=[low,high],wheel_updates=sum(e['kind']=='wheel' and e['update'].get('accepted',False) for e in inside),optical_updates=sum(e['kind']=='optical' and e['update'].get('accepted',False) for e in inside),first_optical_recovery_after_end_s=(after[0]['header_ns']-origin)*1e-9-high if after else None))
   r['controlled_faults']=fault
 runs[n]=r
ay=json.loads((a.ay_root/'XY_lap_metrics.json').read_text());runs['AY_XY_lap']=dict(sensitivity={k:v['historical_common_mask'] for k,v in ay['sensitivity'].items()},corners75=ay['sensitivity']['0.075']['historical_common_mask']['corners'])
tx=read('S_profile_lap/transactions.json');ref=read('S_lap/transactions.json');parity={k:float(max(np.max(abs(np.asarray(f(x))-np.asarray(f(y)))) for x,y in zip(tx,ref))) for k,f in dict(seed=lambda r:r['seed'],P=lambda r:r['prior']['P'],NDT=lambda r:r['ndt']['final'],K=lambda r:r['update']['K'],delta=lambda r:r['update']['delta']).items()};probe=read('S_profile_lap/runtime_probe.json');frames=probe['frames'];gcs=[x for x in probe['gc_events'] if x['frame']>=0];perf=dict(full_cycle_ms=stats([x['cycle_ms'] for x in frames]),over50ms=sum(x['cycle_ms']>50 for x in frames),frames=len(frames),inclusive_modules={k:stats(v) for k,v in probe['modules'].items()},GC_max_in_loop_ms=max([x['ms'] for x in gcs],default=0),peak_rss_mib=max(x['maxrss_kib'] for x in frames)/1024,profile_parity=parity)
e=np.load(a.root/'S_fault_lap/evaluation/evaluation.npz');b=np.load(a.root/'S_lap/evaluation/evaluation.npz');fault_delta=stats(np.linalg.norm(e['candidate']-b['candidate'],axis=1));out=dict(qualification='Conditional GNSS engineering reference only; all new proposals fail overall no-regression',runs=runs,sensor_study=read('sensor_study.json'),raw_wheels=read('wheel_audit.json'),profile=perf,fault_vs_no_fault_position_difference=fault_delta,GNSS_runtime_input=False)
(a.output/'aggregate.json').write_text(json.dumps(out,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,4));x=np.arange(10);base=runs['M_lap']['corners75'];ax.plot(x,[z['baseline']['abs_lateral']['p95'] for z in base],'ko-',label='Original wheel X')
for n,label in [('AY_XY_lap','AY optical XY'),('X_lap','Optical X ablation'),('M_lap','Managed X'),('S_lap','Managed planar speed')]:ax.plot(x,[z['candidate']['abs_lateral']['p95'] for z in runs[n]['corners75']],'o-',label=label)
ax.set(xlabel='Fixed AS segment',ylabel='Lateral reference P95 (m)',xticks=x,title='Fixed75ms engineering reference, not certified truth');ax.legend();fig.tight_layout();fig.savefig(a.output/'corners.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4))
for n,label in [('M_lap','Managed X'),('S_lap','Managed speed'),('AY_XY_lap','AY XY')]:axes[0].plot([0,75,150],[runs[n]['sensitivity'][k]['candidate']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label=label)
axes[0].plot([0,75,150],[runs['M_lap']['sensitivity'][k]['baseline']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'ko-',label='Original');axes[0].set(xlabel='Reference shift(ms)',ylabel='XY reference RMSE(m)');axes[0].legend()
for n,label in [('M_multilap','Managed X'),('S_multilap','Managed speed')]:axes[1].plot([1,2,3],[q['candidate']['XY']['rmse'] for q in runs[n]['laps75']],'o-',label=label)
axes[1].plot([1,2,3],[q['baseline']['XY']['rmse'] for q in runs['M_multilap']['laps75']],'ko-',label='Original');axes[1].set(xlabel='Continuous lap, no reset',ylabel='XY reference RMSE(m)',xticks=[1,2,3]);axes[1].legend();fig.tight_layout();fig.savefig(a.output/'reference_multilap.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4));keys=['accelerating','braking','turning','straight','fast'];axes[0].bar(keys,[out['sensor_study'][k]['reference_compensated_disagreement']['rmse'] for k in keys]);axes[0].tick_params(axis='x',rotation=25);axes[0].set(ylabel='Source disagreement RMS(m/s)',title='Conditional sensor-only compensation, not slip truth');axes[1].hist([z['cycle_ms'] for z in frames],bins=60);axes[1].axvline(50,color='r');axes[1].set(xlabel='Full offline cycle(ms)',ylabel='Frames');fig.tight_layout();fig.savefig(a.output/'sensor_performance.png',dpi=150);plt.close(fig)
print(json.dumps(dict(profile=perf,faults=runs['S_fault_lap']['controlled_faults']),indent=2))
