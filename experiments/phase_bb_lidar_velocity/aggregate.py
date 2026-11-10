"""Aggregate BB results without publishing poses, TF, raw points or sensor traces."""
import argparse,json,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--ay-root',type=Path,required=True);p.add_argument('--ba-root',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(exist_ok=True,parents=True)
def read(p):return json.loads(Path(p).read_text())
def q(v):
 return dict(zip(['p50','p95','p99','max'],map(float,np.percentile(v,[50,95,99,100])))) if len(v) else {}
out=dict(qualification='CONDITIONAL engineering reference; NOT_VALIDATED independent speed/accuracy/covariance; overall repair FAILED',runs={},frozen_pairs=read(a.root/'frozen_velocity.json'),source_study=read(a.root/'source_study.json'),peak_checks=read(a.root/'peak_checks.json'),final_prefix_parity=read(a.root/'final_prefix_parity.json'))
for n in ['P_lap','L_lap','P_multilap','P_final_multilap','P_fault','R_lap','R_multilap']:
 d=read(a.root/(n+'_metrics.json'));tx=read(a.root/n/'transactions.json');rows=[json.loads(s) for s in (a.root/n/'lidar_velocity.jsonl').open()];probe=read(a.root/n/'runtime_probe.json');profile=[r['cycle_ms'] for r in probe['frames']];pairs=[p for r in rows for p in r['pairs']];used=[r['ndt']['velocity_auxiliary'] for r in tx if r['ndt']['velocity_auxiliary']['applied']];ages=[(t['complete']-v['midpoint_ns'])*1e-6 for t,v in zip(tx,rows) if v.get('qualified')];r={k:d[k] for k in ['processed_frames','accepted','rejected','duration_s','NDT_converged','P_min_eigenvalue','P_symmetry_max','unit_lateral_gain','lateral_innovation','actual_body_lateral_update','Z_span_m','vz_range','bg_norm','ba_norm','max_gap_s']};r.update(sensitivity={k:v['historical_common_mask'] for k,v in d['sensitivity'].items()},laps75=d['per_preselected_lap_75ms_raw_reference'],cycle_ms=q(profile),cycle_over50ms=int(sum(x>50 for x in profile)),velocity_frontend_ms=q([v['frontend_wall_ms'] for v in rows]),velocity_qualified=sum(v['qualified'] for v in rows),velocity_usage_counts=dict(collections.Counter(v['usage'] for v in rows)),pair_quality_counts=dict(collections.Counter(v['quality'] for v in pairs)),auxiliary_applied=len(used),auxiliary_shift_norm_m=q([np.linalg.norm(v['delta_xy']) for v in used]),modelled_velocity_age_ms=q(ages),service_failure_count=sum('service_failure' in v for v in rows),peak_rss_MiB=max(v['maxrss_kib'] for v in probe['frames'])/1024,transactions_sha256=read(a.root/(n+'_summary.json'))['transactions_sha256']);before=[];after=[]
 for z in tx:
  aux=z['ndt']['velocity_auxiliary']
  if not aux['applied']:continue
  e=np.array(z['seed'])[:2,3]-np.array(z['ndt']['final'])[:2,3];before.append(float(np.linalg.norm(e)));after.append(float(np.linalg.norm(e+aux['delta_xy'])))
 r['distance_to_own_NDT_endpoint']=dict(before=q(before),after=q(after),fraction_improved=float(np.mean(np.array(after)<np.array(before))),qualification='optimizer endpoint difference, not independent position accuracy');out['runs'][n]=r
aytx=read(a.ay_root/'XY_lap/transactions.json')
for name in ['P_lap','R_lap']:
 tx=read(a.root/name/'transactions.json');r=out['runs'][name];r['actual_feedback_difference_from_AY']=dict(NDT_XY_norm=q([np.linalg.norm(np.asarray(x['ndt']['final'])[:2,3]-np.asarray(y['ndt']['final'])[:2,3]) for x,y in zip(tx,aytx)]),post_velocity_norm=q([np.linalg.norm(np.asarray(x['prior']['v'])+np.asarray(x['update']['delta'])[3:6]-np.asarray(y['prior']['v'])-np.asarray(y['update']['delta'])[3:6]) for x,y in zip(tx,aytx)]),gyro_bias_norm=q([np.linalg.norm(np.asarray(x['prior']['bg'])+np.asarray(x['update']['delta'])[9:12]-np.asarray(y['prior']['bg'])-np.asarray(y['update']['delta'])[9:12]) for x,y in zip(tx,aytx)]),accelerometer_bias_norm=q([np.linalg.norm(np.asarray(x['prior']['ba'])+np.asarray(x['update']['delta'])[12:15]-np.asarray(y['prior']['ba'])-np.asarray(y['update']['delta'])[12:15]) for x,y in zip(tx,aytx)]),qualification='same native time, actual own-state outputs; differences not accuracy')
ay=read(a.ay_root/'XY_lap_metrics.json');ba=read(a.ba_root/'B_lap_metrics.json');out['AY']=dict(sensitivity={k:v['historical_common_mask'] for k,v in ay['sensitivity'].items()},laps75=read(a.ay_root/'XY_multilap_metrics.json')['per_preselected_lap_75ms_raw_reference']);out['BA']=dict(sensitivity={k:v['historical_common_mask'] for k,v in ba['sensitivity'].items()})
rows=[json.loads(s) for s in (a.root/'P_fault/lidar_velocity.jsonl').open()];origin=rows[0]['end_ns'];within=[r for r in rows if 20<=(r['end_ns']-origin)*1e-9<22];after=[r for r in rows if (r['end_ns']-origin)*1e-9>=22 and r['qualified']];out['fault_injection']=dict(kind='velocity-output unavailable only, not global LiDAR loss',interval_s=[20,22],samples=len(within),qualified_within=sum(r['qualified'] for r in within),first_qualified_after_end_s=(after[0]['end_ns']-origin)*1e-9-22 if after else None)
# Monitor-mode full physical15D/P parity against AY at all40 native times.
mon=read(a.root/'M40/transactions.json');ref=read(a.ay_root/'XY_lap/transactions.json')[:40];out['monitor40_parity']={k:max(float(np.max(abs(np.array(f(x))-np.array(f(y))))) for x,y in zip(mon,ref)) for k,f in dict(seed=lambda r:r['seed'],P=lambda r:r['prior']['P'],NDT=lambda r:r['ndt']['final'],delta=lambda r:r['update']['delta']).items()}
old=[json.loads(s) for s in (a.root/'frozen_pairs.jsonl').open()];new=[json.loads(s) for s in (a.root/'rebuilt_pairs.jsonl').open()];out['fresh_binary_reproduction']=dict(pairs=len(new),pose_max_difference=max(float(np.max(abs(np.array(x['final'])-np.array(y['final'])))) for x,y in zip(old,new)),H_max_difference=max(float(np.max(abs(np.array(x['H_right_rotation_translation'])-np.array(y['H_right_rotation_translation'])))) for x,y in zip(old,new)))
(a.out/'aggregate.json').write_text(json.dumps(out,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,4));xx=np.arange(10);base=out['runs']['P_lap']['sensitivity']['0.075']['corners'];ax.plot(xx,[r['baseline']['abs_lateral']['p95'] for r in base],'ko-',label='Original')
for label,corners in [('AY',out['AY']['sensitivity']['0.075']['corners']),('BA',out['BA']['sensitivity']['0.075']['corners']),('BB20Hz',base),('BB10Hz',out['runs']['L_lap']['sensitivity']['0.075']['corners']),('BB feedback-aware',out['runs']['R_lap']['sensitivity']['0.075']['corners'])]:ax.plot(xx,[r['candidate']['abs_lateral']['p95'] for r in corners],'o-',label=label)
ax.set(xlabel='Fixed segment',ylabel='Lateral GNSS-reference P95(m)',xticks=xx,title='Fixed75ms engineering reference, not certified truth');ax.legend();fig.tight_layout();fig.savefig(a.out/'corners.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4))
for n,label in [('P_final_multilap','BB20Hz'),('R_multilap','BB feedback-aware')]:
 laps=out['runs'][n]['laps75'];axes[0].plot([1,2,3],[r['candidate']['XY']['rmse'] for r in laps],'o-',label=label);
 if n=='P_final_multilap':axes[0].plot([1,2,3],[r['baseline']['XY']['rmse'] for r in laps],'ko-',label='Original')
axes[0].plot([1,2,3],[r['candidate']['XY']['rmse'] for r in out['AY']['laps75']],'o-',label='AY');axes[0].set(xlabel='Continuous lap',ylabel='XY reference RMSE(m)',xticks=[1,2,3]);axes[0].legend()
for n,label in [('P_lap','BB20Hz'),('L_lap','BB10Hz')]:axes[1].plot([0,75,150],[out['runs'][n]['sensitivity'][k]['candidate']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label=label)
axes[1].plot([0,75,150],[out['AY']['sensitivity'][k]['candidate']['XY']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label='AY');axes[1].set(xlabel='Predeclared reference shift(ms)',ylabel='XY reference RMSE(m)');axes[1].legend();fig.tight_layout();fig.savefig(a.out/'reference_multilap.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4));names=list(out['source_study']['groups']);v=[out['source_study']['groups'][k]['lidar_minus_raw']['RMS'] for k in names];axes[0].bar(np.arange(len(names))-.15,[r[0] for r in v],.3,label='X');axes[0].bar(np.arange(len(names))+.15,[r[1] for r in v],.3,label='Y');axes[0].set(xticks=np.arange(len(names)),xticklabels=names,ylabel='LiDAR - optical interval RMS(m/s)',title='Source differences, not speed accuracy');axes[0].tick_params(axis='x',rotation=30);axes[0].legend()
for n,label in [('P_lap','20Hz lap'),('L_lap','10Hz lap'),('P_final_multilap','20Hz3laps')]:
 probe=read(a.root/n/'runtime_probe.json');v=np.sort([r['cycle_ms'] for r in probe['frames']]);axes[1].plot(v,np.arange(1,len(v)+1)/len(v),label=label)
axes[1].axvline(50,color='k',ls='--');axes[1].set(xlabel='Full offline cycle(ms)',ylabel='Empirical CDF',xscale='log',title='No ROS2 online qualification');axes[1].legend();fig.tight_layout();fig.savefig(a.out/'speed_latency.png',dpi=150);plt.close(fig)
# Full native speed/time plot stays private.
s=read(a.root/'source_study.private.json');t=(np.array([r['header_ns'] for r in s])-s[0]['header_ns'])*1e-9;fig,axes=plt.subplots(2,1,figsize=(12,6))
for j,ax in enumerate(axes):
 for k in ['lidar_sensor','raw','filtered','IMU_prediction_sensor']:ax.plot(t,[r[k][j] for r in s],label=k)
 ax.set(ylabel='Sensor-axis velocity(m/s)',xlabel='Relative native time(s)');ax.legend()
fig.tight_layout();fig.savefig(a.root/'speed_sources.private.png',dpi=150);plt.close(fig)
print(json.dumps({n:{k:r[k] for k in ['processed_frames','auxiliary_applied','velocity_qualified','cycle_ms','service_failure_count']} for n,r in out['runs'].items()},indent=2))
