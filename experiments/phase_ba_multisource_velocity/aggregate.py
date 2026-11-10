"""Publish aggregate/error figures only: no trajectory, TF or raw sensor payload."""
import argparse,json,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--ay-root',type=Path,required=True);p.add_argument('--az-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
def read(n):return json.loads((a.root/n).read_text())
def stats(v):
 v=np.asarray(v);return dict(n=len(v),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),p99=float(np.percentile(v,99)),max=float(np.max(v))) if len(v) else dict(n=0)
runs={}
for n in ['F_lap','B_lap','J_lap','C_lap','W_lap','B_multilap','J_multilap','B_fault_lap']:
 d=read(n+'_metrics.json');r={k:d[k] for k in ['processed_frames','accepted','rejected','duration_s','NDT_converged','P_min_eigenvalue','P_symmetry_max','unit_lateral_gain','lateral_innovation','actual_body_lateral_update','Z_span_m','vz_range','bg_norm','ba_norm','performance','max_gap_s','strict_repair_screen']};r['transactions_sha256']=read(n+'_summary.json')['transactions_sha256'];r['sensitivity']={k:{'valid':v['historical_common_mask']['valid'],'coverage':v['historical_common_mask']['coverage'],**{z:v['historical_common_mask'][z] for z in ['candidate','baseline','old_EKF']}} for k,v in d['sensitivity'].items()};r['corners75']=d['sensitivity']['0.075']['historical_common_mask']['corners'];r['laps75']=d['per_preselected_lap_75ms_raw_reference']
 f=a.root/n/'optical_updates.jsonl'
 if f.exists():
  last={}
  for line in f.open():e=json.loads(line);last[e['message_id']]=e['update']
  r['optical_decisions']=dict(collections.Counter(e.get('reason','UNKNOWN') for e in last.values()));r['frontend_states']=dict(collections.Counter(e.get('frontend_used','SHARED_RAW_XY') for e in last.values()));r['braking_flags']=dict(collections.Counter(str(e.get('braking','not-applicable')) for e in last.values()))
 w=json.loads((a.root/n/'wheel.json').read_text());last_w={e['header_ns']:e for e in w};r['wheel_unique_headers']=len(last_w);r['wheel_decisions']=dict(collections.Counter(e['update'].get('reason','UNKNOWN') for e in last_w.values()))
 runs[n]=r
for name,root,file in [('AY',a.ay_root,'XY_lap_metrics.json'),('AZ',a.az_root,'M_lap_metrics.json')]:
 d=json.loads((root/file).read_text());runs[name]=dict(sensitivity={k:v['historical_common_mask'] for k,v in d['sensitivity'].items()},corners75=d['sensitivity']['0.075']['historical_common_mask']['corners'])
x=read('B_lap/transactions.json');y=read('B_guard_profile/transactions.json');parity={k:max(float(np.max(abs(np.array(f(i))-np.array(f(j))))) for i,j in zip(x,y)) for k,f in dict(seed=lambda r:r['seed'],P=lambda r:r['prior']['P'],NDT=lambda r:r['ndt']['final'],K=lambda r:r['update']['K'],delta=lambda r:r['update']['delta']).items()};assert max(parity.values())==0
probe=read('B_guard_profile/runtime_probe.json');v=[f['cycle_ms'] for f in probe['frames']];profile=dict(full_cycle_ms=stats(v),over50ms=sum(x>50 for x in v),frames=len(v),inclusive_modules_ms={k:stats(v) for k,v in probe['modules'].items()},parity=parity,peak_rss_MiB=max(x['maxrss_kib'] for x in probe['frames'])/1024)
radar=read('radar_feasibility.json');raw=read('radar_feasibility.private.json')
for key,val in radar.items():val['header_to_record_ms_quantiles']=np.percentile([(e['recorder_ns']-e['header_ns'])*1e-6 for e in raw[key]],[0,50,95,100]).tolist()
causal=read('causal_analysis.json');out=dict(qualification='Engineering GNSS reference only; no overall repair or certified consistency',runs=runs,profile=profile,frontend_dynamics=read('frontend_dynamics.json'),frontend_real_audit=read('frontend_audit.json'),frontend_CPU_and_byte_reproduction=read('frontend_reproduce_audit.json'),query_time_proxy=read('query_time_audit.json'),pre_run_joint_noise=read('joint_audit.json'),block_audit=read('blocks_audit.json'),causal=causal,brake_command=read('brake_command_audit.json'),auxiliary=read('auxiliary_check.json'),radar=radar,GNSS_runtime=False,lap_reset=False)
fault_rows={}
for line in (a.root/'B_fault_lap/optical_updates.jsonl').open():
 z=json.loads(line);fault_rows[z['message_id']]=z
raw=json.loads((a.ay_root/'optical.json').read_text())['events'];origin=min(e['header_ns'] for e in raw)+1_000_000_000;interval=[]
for lo,hi in [(20,22),(35,35.5)]:
 inside=[z for z in fault_rows.values() if lo<=(z['header_ns']-origin)*1e-9<hi];after=sorted([z for z in fault_rows.values() if (z['header_ns']-origin)*1e-9>=hi and z['update'].get('accepted')],key=lambda z:z['header_ns']);interval.append(dict(interval_s=[lo,hi],accepted_optical_inside=sum(z['update'].get('accepted',False) for z in inside),first_accepted_after_end_s=(after[0]['header_ns']-origin)*1e-9-hi if after else None))
out['fault_injection']=interval
(a.output/'aggregate.json').write_text(json.dumps(out,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,4));xx=np.arange(10);ax.plot(xx,[q['baseline']['abs_lateral']['p95'] for q in runs['B_lap']['corners75']],'ko-',label='Original')
for n,label in [('AY','AY'),('F_lap','Polynomial XY'),('B_lap','Brake-only X'),('J_lap','Joint shared error'),('C_lap','Non-overlap blocks'),('W_lap','Brake wheel omission')]:ax.plot(xx,[q['candidate']['abs_lateral']['p95'] for q in runs[n]['corners75']],'o-',label=label)
ax.set(xlabel='Fixed AS segment',ylabel='Lateral reference P95(m)',xticks=xx,title='Frozen75ms engineering reference, not certified truth');ax.legend(fontsize=8,ncol=2);fig.tight_layout();fig.savefig(a.output/'corners.png',dpi=150);plt.close(fig)
pr=np.load(a.root/'frontend_probes.npz');fig,axes=plt.subplots(1,2,figsize=(10,4))
for label in ['legacy','PSD']:
 axes[0].semilogx(pr['frequencies_Hz'],20*np.log10(pr[label+'_gain']),label=label);t=pr['time_s'];sel=(t>=1.8)&(t<3.5);axes[1].plot(t[sel]-2,pr[label+'_step'][sel],label=label)
axes[0].set(xlabel='Frequency(Hz), nominal250Hz input',ylabel='Magnitude(dB)',ylim=[-30,8]);axes[0].axhline(-3,color='gray',ls='--');axes[1].plot([-.2,0,0,1.5],[20,20,10,10],'k--',label='input');axes[1].set(xlabel='Time from synthetic step(s)',ylabel='Velocity(m/s)');axes[0].legend();axes[1].legend();fig.suptitle('Fixed source parameters; corrected Q is not the dominant response change');fig.tight_layout();fig.savefig(a.output/'frontend_response.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4))
for n,label in [('AY','AY'),('B_lap','Brake-only X'),('F_lap','Polynomial XY'),('J_lap','Joint')]:axes[0].plot([0,75,150],[runs[n]['sensitivity'][k]['candidate']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label=label)
axes[0].plot([0,75,150],[runs['B_lap']['sensitivity'][k]['baseline']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'ko-',label='Original');axes[0].set(xlabel='Predeclared reference shift(ms)',ylabel='XY reference RMSE(m)');axes[0].legend(fontsize=8)
for n,label in [('B_multilap','Brake-only X'),('J_multilap','Joint')]:axes[1].plot([1,2,3],[v['candidate']['XY']['rmse'] for v in runs[n]['laps75']],'o-',label=label)
axes[1].plot([1,2,3],[v['baseline']['XY']['rmse'] for v in runs['B_multilap']['laps75']],'ko-',label='Original');axes[1].set(xlabel='Continuous lap',ylabel='XY reference RMSE(m)',xticks=[1,2,3]);axes[1].legend();fig.tight_layout();fig.savefig(a.output/'reference_multilap.png',dpi=150);plt.close(fig)
# Plot only errors and relative time; no map coordinates or reconstructive poses.
ser=np.load(a.root/'causal_series.npz');ev=np.load(a.root/'B_lap/evaluation/evaluation.npz');ns=ser['native_header'];c=ser['corner'];fig,axes=plt.subplots(2,2,figsize=(11,6))
for ax,corner in zip(axes.flat,[3,9,5,8]):
 ids=np.flatnonzero(c==corner);origin=ns[ids[0]];mask=(ns>=origin-3_000_000_000)&(ns<=ns[ids[-1]]+500_000_000);t=(ns-origin)*1e-9;valid=ev['historical_valid_0.075'];
 for name,label in [('B_lap','Brake-only X'),('W_lap','Wheel omission'),('J_lap','Joint')]:v=ser[name+'_posterior_reference_lateral'].copy();v[~valid]=np.nan;ax.plot(t[mask],v[mask],label=label)
 ax.axvline(0,color='k',ls='--');ax.set(title='Fixed segment '+str(corner),xlabel='Seconds from fixed segment entry',ylabel='Signed lateral reference difference(m)');ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(a.output/'corner_causality.png',dpi=150);plt.close(fig)
print(json.dumps(dict(profile=profile,pre_intervention=causal['B_lap']['intervention']),indent=2))
