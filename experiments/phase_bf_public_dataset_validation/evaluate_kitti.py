"""Offline reference evaluation only; never imported by the online runner."""
import argparse,csv,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
spec=importlib.util.spec_from_file_location('kitti_adapter',Path(__file__).with_name('kitti_adapter.py'));a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

def stats(x):
 x=np.asarray(x,float);return dict(n=len(x),p50=float(np.median(x)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),max=float(np.max(x)))
def rmse(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def poses(rows,key):
 T=np.repeat(np.eye(4)[None],len(rows),axis=0)
 for j,r in enumerate(rows):T[j,:3,:3]=r[key]['R'];T[j,:3,3]=r[key]['p']
 return T

def compare(T,t,vel,online,ref,offset):
 times=np.asarray(t,dtype=np.int64)+round(offset*1e9);valid=(times>=ref['raw_t'][0])&(times<=ref['raw_t'][-1]);tv=times[valid]
 G=a.reference_at(ref['raw_t'],ref['raw_packets'],np.r_[online['scan_t'][0],tv])[1:];E=T[valid];err=E[:,:3,3]-G[:,:3,3]
 heading=G[:,:2,0];heading/=np.linalg.norm(heading,axis=1)[:,None];normal=np.column_stack((-heading[:,1],heading[:,0]))
 longitudinal=np.einsum('ij,ij->i',err[:,:2],heading);lateral=np.einsum('ij,ij->i',err[:,:2],normal)
 yawG=np.arctan2(G[:,1,0],G[:,0,0]);yawE=np.arctan2(E[:,1,0],E[:,0,0]);dyaw=np.arctan2(np.sin(yawE-yawG),np.cos(yawE-yawG))
 rel=[]
 for i in range(len(tv)):
  j=int(np.argmin(abs((tv-tv[i])*1e-9-1.)))
  if j>i and abs((tv[j]-tv[i])*1e-9-1)<.06:
   d=np.linalg.inv(np.linalg.inv(G[i])@G[j])@(np.linalg.inv(E[i])@E[j]);rel.append([np.linalg.norm(d[:3,3]),np.linalg.norm(Rotation.from_matrix(d[:3,:3]).as_rotvec())])
 result=dict(n=int(valid.sum()),XY_RMSE_m=rmse(np.linalg.norm(err[:,:2],axis=1)),ATE3D_common_initial_frame_RMSE_m=rmse(np.linalg.norm(err,axis=1)),
    lateral_abs_m=stats(abs(lateral)),longitudinal_RMSE_m=rmse(longitudinal),yaw_RMSE_deg=rmse(np.rad2deg(dyaw)),Z_reference_RMSE_m=rmse(err[:,2]),
    RPE_1s_pairs=len(rel),RPE_1s_translation_RMSE_m=rmse(np.array(rel)[:,0]) if rel else None,RPE_1s_rotation_RMSE_deg=rmse(np.rad2deg(np.array(rel)[:,1])) if rel else None)
 if vel is not None:
  vu=np.interp((tv-ref['raw_t'][0]).astype(float),(ref['raw_t']-ref['raw_t'][0]).astype(float),ref['raw_packets'][:,10]);result['Vz_reference_RMSE_m_s']=rmse(np.asarray(vel)[valid,2]-vu)
 return result,(G,err,lateral,longitudinal,dyaw)

def main(root,results,out,b1_run="B1_run1"):
 root=Path(root);results=Path(results);out=Path(out);out.mkdir(exist_ok=False)
 online=np.load(root/'online.npz');ref=np.load(root/'offline_reference.npz');allmetrics={};plots={}
 for mode in ['B0','B1']:
  run=results/(b1_run if mode=='B1' else 'B0_run1');rows=[json.loads(s) for s in (run/'frames.jsonl').read_text().splitlines()];t=np.array([r['actual_published_state']['t_ns'] for r in rows],dtype=np.int64);T=poses(rows,'actual_published_state');vel=np.array([r['actual_published_state']['v'] for r in rows]);metrics={}
  for offset in [0.,.075,.15]:
   v,diagnostic=compare(T,t,vel,online,ref,offset);metrics[str(offset)]=v
   if offset==0:plots[mode]=(rows,T,diagnostic)
  M=np.array([r['ndt']['final'] for r in rows]);mt=np.array([r['scan_epoch_ns'] for r in rows],dtype=np.int64)
  metrics['NDT_all_proposals_reference0']=compare(M,mt,None,online,ref,0)[0]
  take=np.array([r['ndt']['converged'] and r['ndt']['candidate_valid'] for r in rows]);metrics['NDT_converged_only_reference0']=compare(M[take],mt[take],None,online,ref,0)[0]
  profiles=[json.loads(s) for s in (run/'cycle_profile.jsonl').read_text().splitlines()]
  perf={key:stats([r[key] for r in rows]) for key in ['service_ms','prepare_and_prior_ms','input_decode_transform_ms','rollback_replay_ms','cycle_ms','epoch_to_available_ms']}
  perf['instrumented_full_cycle_ms']=stats([r['instrumented_full_cycle_ms'] for r in profiles]);perf['over50_count']=sum(r['over_50ms'] for r in profiles);perf['ndt_align_wall_ms']=stats([r['ndt']['align_wall_ms'] for r in rows]);perf['extra_quality_ms']=stats([r['ndt']['quality_extra_ms'] for r in rows]);perf['startup_ms']=json.loads((run/'startup.json').read_text())['ms']
  nis=np.array([r['actual_update'].get('conditional_NIS',np.nan) for r in rows]);accepted=sum(r['actual_update']['accepted'] for r in rows)
  metrics.update(completed=len(rows),accepted=accepted,rejected=len(rows)-accepted,converged=int(take.sum()),max_iterations=sum(r['ndt']['exit']=='MAX_ITERATIONS' for r in rows),
    iteration_stats=stats([r['ndt']['iterations'] for r in rows]),conditional_NIS=stats(nis[np.isfinite(nis)]),performance=perf,
    A1_abs_shadow_m_s=stats([abs(r['A_shadow']['A1']['residual']) for r in rows]),A2_abs_shadow_m_s=stats([abs(r['A_shadow']['A2']['residual']) for r in rows]),
    Vz_state_m_s=stats(abs(vel[:,2])))
  frozen={}
  for branch in ['B0','B1']:
   valid=[r['frozen'][branch] for r in rows if 'K' in r['frozen'][branch]]
   frozen[branch]=dict(available=len(valid),all_shadow=True,abs_dvz_proposed=stats([abs(r['delta_proposed'][5]) for r in valid]),
        domain_failure_count=sum(bool(r['original_domain_failures']) for r in valid),sigma_xyz=np.sqrt(np.median([np.diag(r['R'])[:3] for r in valid],axis=0)).tolist(),
        sigma_rotation=np.sqrt(np.median([np.diag(r['R'])[3:] for r in valid],axis=0)).tolist(),nonzero_cross_terms=sum(np.any(abs(np.asarray(r['R'])-np.diag(np.diag(r['R'])))>1e-15) for r in valid))
  metrics['same_prior_frozen_on_this_run']=frozen
  with (out/f'{mode}_metrics.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['frame','observation_stamp_ns','published_stamp_ns','published_x','published_y','published_z','published_vz','accepted','converged','fitness','full_cycle_ms'])
   for r,p in zip(rows,profiles):
    s=r['actual_published_state'];w.writerow([r['frame'],r['scan_epoch_ns'],s['t_ns'],*s['p'],s['v'][2],r['actual_update']['accepted'],r['ndt']['converged'],r['ndt']['fitness'],p['instrumented_full_cycle_ms']])
  allmetrics[mode]=metrics
 (out/'aggregate_results.json').write_text(json.dumps(allmetrics,indent=2,default=lambda x:x.item())+'\n')
 # Scientific plots are generated offline and are never consumed by runtime.
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(2,3,figsize=(14,8))
 for mode,(rows,T,(G,err,lat,longi,dyaw)) in plots.items():
  ids=[r['frame'] for r in rows];ax[0,0].plot(T[:,0,3],T[:,1,3],label=mode);ax[0,1].plot(ids,np.linalg.norm(err[:,:2],axis=1),label=mode);ax[0,2].plot(ids,np.rad2deg(dyaw),label=mode)
  ax[1,0].plot(ids,err[:,2],label=mode);ax[1,1].plot(ids,[r['actual_published_state']['v'][2] for r in rows],label=mode);ax[1,2].plot(ids,[r['ndt']['fitness'] for r in rows],label=mode)
 ax[0,0].plot(G[:,0,3],G[:,1,3],'k--',label='OXTS reference')
 for v,title in zip(ax.ravel(),['Map XY (m), published-time comparison','XY reference discrepancy (m)','Yaw reference discrepancy (deg)','Z reference discrepancy (m)','Published Vz (m/s)','PCL-like fitness (m2)']):v.set_title(title);v.legend();v.grid(True)
 fig.suptitle('KITTI0002: fixed map0–19 / test20–76; reference-assisted functional validation, not independent truth')
 fig.tight_layout();fig.savefig(out/'diagnostics.png',dpi=150);plt.close(fig)
 print(json.dumps({k:{'XY0':v['0.0']['XY_RMSE_m'],'XY75':v['0.075']['XY_RMSE_m'],'XY150':v['0.15']['XY_RMSE_m'],'accepted':v['accepted'],'full_cycle':v['performance']['instrumented_full_cycle_ms'],'over50':v['performance']['over50_count']} for k,v in allmetrics.items()},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--results',required=True);p.add_argument('--out',required=True);p.add_argument('--b1-run',default='B1_run1');v=p.parse_args();main(v.root,v.results,v.out,v.b1_run)
