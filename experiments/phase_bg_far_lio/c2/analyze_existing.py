"""BG-C2 READ ONLY: descriptive statistics/algebra from saved BG-C1 logs.
No matcher, predictor, corrector, replay, ROS graph or parameter writer is imported.
All per-frame derivatives stay private; public output contains aggregates only.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ORIGIN=1763205419863639180
WINDOWS={'pit':(135,180),'drive':(183,345.806063080),'normal8':(307.30626695,313.70623555),'right3':(339.406271902,343.806063080)}

def read(p):
 with Path(p).open() as f:
  for l in f:yield json.loads(l)
def stats(x):
 a=np.asarray(x,float);a=a[np.isfinite(a)]
 if not len(a):return dict(n=0)
 return dict(n=len(a),min=float(a.min()),max=float(a.max()),mean=float(a.mean()),p50=float(np.median(a)),p95=float(np.percentile(abs(a),95)),rmse=float(np.sqrt(np.mean(a*a))))
def corr(a,b):
 a=np.asarray(a);b=np.asarray(b)
 return float(np.corrcoef(a,b)[0,1]) if len(a)>3 and np.std(a)>1e-15 and np.std(b)>1e-15 else None

def main(a):
 root=Path(a.root);out=Path(a.out);out.mkdir(exist_ok=False);ref=np.load(Path(a.be)/'runtime/reference_lap.npz');fixed=np.load(Path(a.data)/'phase_as_accuracy_quality/single_lap/evaluation.npz');lookup={int(t):i for i,t in enumerate(ref['header'])}
 D=[];far_cov=[];far_states=[]
 for r in read(root/'far_recorded_init/records.jsonl'):
  if r['kind']=='timing_debug' and r['node']=='StateEstimation':D.append(r)
  elif r['kind']=='lidar':far_cov.append(np.array(r['pose_cov']).reshape(6,6))
  elif r['kind']=='ekf':far_states.append(r)
 Rkeys=['kalman_filter/lidar_gicp/R_'+s for s in ['x_m','y_m','z_m','phi_rad','theta_rad','psi_rad']];bkeys=['bias/'+s for s in ['dphi_rads','dtheta_rads','dpsi_rads','ax_mps2','ay_mps2','az_mps2']]
 result=dict(scope='saved C1 only; no new filter/registration; source algebra and descriptive association, not causal ablation',FAR={},new15D={},lateral={},checks={},unknown=['FAR full9x9 P/K/H/S and post-clipping residual per measurement ID not saved','Full nonlinear observability rank with map/data association errors not certified'])
 norms=np.array([np.linalg.norm(v) for v in far_cov]);result['FAR']['input_pose_covariance_frobenius']=stats(norms);result['FAR']['input_pose_covariance_nonzero_count']=int((norms!=0).sum());result['FAR']['debug_count']=len(D)
 for name,(lo,hi) in WINDOWS.items():
  d=[r for r in D if lo<=(r['sim_ns']-ORIGIN)*1e-9<=hi];v=[r['values'] for r in d];z={}
  z['snapshot_count']=len(v);z['time_contract']='debug receipt simulation time, stamp0; not exact per-observation timestamp'
  z['R_diagonal']={k:stats([x[k] for x in v if k in x]) for k in Rkeys}
  z['R_distinct_values']={k:len(set(x[k] for x in v if k in x)) for k in Rkeys}
  z['validity']={k:stats([x[k] for x in v if k in x]) for k in ['state_machine/valid_position/lidar_gicp','state_machine/valid_orientation/lidar_gicp','state_machine/valid_velocity/lidar_gicp','state_machine/valid_imu/primary']}
  z['bias']={k:stats([x[k] for x in v if k in x]) for k in bkeys}
  z['bias_first_last']={k:[v[0].get(k),v[-1].get(k)] for k in bkeys} if v else {}
  z['vehicle_model_unfed_debug']={k:stats([x[k] for x in v if k in x]) for k in ['model/vx_mps','model/vy_mps']}
  z['tilt_reference']={k:stats([x[k] for x in v if k in x]) for k in ['ref_angles/phi','ref_angles/theta']}
  live=[x for x in v if x.get('pose/lidar_gicp/virtual/msg_delay',0)>0]
  z['measurement_debug_live_snapshots']=len(live)
  for key in ['pose/lidar_gicp/measured/msg_delay','pose/lidar_gicp/virtual/msg_delay']:
   z[key]=stats([x[key]*1000 for x in live if key in x])
  diffs=[]
  for x in live:
   p=np.array([x.get('pose/lidar_gicp/virtual/position_m/'+k,0)-x.get('pose/lidar_gicp/measured/position_m/'+k,0) for k in ['x','y','z']]);diffs.append(np.linalg.norm(p))
  z['virtual_minus_measured_compensated_position_norm']=stats(diffs)
  # Raw position residual norm bound proves no ellipsoid clipping if ||r||²/min(bound)<=1.
  res=[np.array([x.get('kalman_filter/lidar_gicp/res_'+k+'_m',0) for k in ['x','y','z']]) for x in v]
  res=np.array(res);z['position_clip_guaranteed_none_snapshots']=int((np.sum(res*res,axis=1)<=1).sum()) if len(res) else 0
  z['position_clip_guaranteed_active_snapshots']=int((np.sum(res*res,axis=1)>5).sum()) if len(res) else 0
  z['position_clip_ambiguous_snapshots']=len(res)-z['position_clip_guaranteed_none_snapshots']-z['position_clip_guaranteed_active_snapshots']
  orient=np.array([[x.get('kalman_filter/lidar_gicp/res_'+k+'_rad',0) for k in ['phi','theta','psi']] for x in v]);z['orientation_clip_norm_exceeds_bound']=int((np.sum(orient*orient,axis=1)>.3).sum()) if len(orient) else 0
  result['FAR'][name]=z
 # Compress saved15D state/P into descriptive traces, never propagate/update them.
 states=[]
 for r in read(root/'eskf15_recorded_init/states.jsonl'):
  t=(r['stamp_ns']-ORIGIN)*1e-9;P=np.array(r['P']);R=Rotation.from_quat(r['quaternion']).as_matrix();bg=np.array(r['bg']);ba=np.array(r['ba']);RP=Rotation.from_matrix(R).as_euler('xyz')
  sigma=np.sqrt(np.maximum(np.diag(P),1e-300));C=P/np.outer(sigma,sigma)
  states.append(dict(t=t,bg=bg,ba=ba,R=R,p=np.array(r['position']),v=np.array(r['velocity_map_imu']),rpy=RP,sigma_bg=sigma[9:12],sigma_ba=sigma[12:15],bias_xy_corr=np.max(abs(C[:2,9:15])),tilt_ba_corr=np.max(abs(C[6:8,12:15]))))
 for name,(lo,hi) in WINDOWS.items():
  q=[r for r in states if lo<=r['t']<=hi];z={}
  for k in ['bg','ba','sigma_bg','sigma_ba']:z[k+'_norm']=stats([np.linalg.norm(r[k]) for r in q])
  for k in ['bias_xy_corr','tilt_ba_corr']:z[k]=stats([r[k] for r in q])
  for k in ['bg','ba']:
   z[k+'_step_norm']=stats([np.linalg.norm(v[k]-u[k]) for u,v in zip(q,q[1:])]);z[k+'_endpoint_change_norm']=float(np.linalg.norm(q[-1][k]-q[0][k])) if q else None
  z['roll_deg']=stats([np.rad2deg(v['rpy'][0]) for v in q]);z['pitch_deg']=stats([np.rad2deg(v['rpy'][1]) for v in q]);result['new15D'][name]=z
 decomposition=[];maxgainerr=0.;max_res_difference=0.
 for r in read(root/'eskf15_recorded_init/matches.jsonl'):
  if 'ndt' not in r or r['stamp_ns'] not in lookup:continue
  ix=lookup[r['stamp_ns']]
  if not(675<=ix<=803 or 1317<=ix<=1405):continue
  u=r['update'];prior=r['prior'];ndt=r['ndt'];seed=np.array(r['seed']);P=np.array(prior['P']);R=np.array(prior['R']);lever=R.T@(np.array(prior['p'])-seed[:3,3]);tan=fixed['tangent'][ix];n=np.r_[-tan[1],tan[0],0.];rxy=np.array(ndt['final'])[:3,3]-seed[:3,3];inn=np.array(u['innovation']);K=np.array(u['K']);delta=np.array(u.get('delta',u.get('delta_proposed')))
  gainerr=np.linalg.norm(K@inn-delta);maxgainerr=max(maxgainerr,gainerr);max_res_difference=max(max_res_difference,float(np.linalg.norm(rxy-inn[:3])))
  # First-order base-point correction from stored K*innovation, including attitude lever arm.
  sk=np.array([[0,-lever[2],lever[1]],[lever[2],0,-lever[0]],[-lever[1],lever[0],0]])
  B=np.zeros((3,15));B[:,:3]=np.eye(3);B[:,6:9]=R@sk;kg=n@B@K;groups=[float(kg[:3]@inn[:3]),float(kg[3:5]@inn[3:5]),float(kg[5]*inn[5])]
  exact_delta=delta[:3]-R@(Rotation.from_rotvec(delta[6:9]).as_matrix()-np.eye(3))@lever
  row=dict(index=int(ix),prior_lateral=float(n@(seed[:3,3]-ref['G_0.075'][ix])),observation_lateral=float(n@(np.array(ndt['final'])[:3,3]-ref['G_0.075'][ix])),correction_lateral_linear=float(sum(groups)),correction_lateral_exact=float(n@exact_delta),position_measurement_contribution=groups[0],roll_pitch_contribution=groups[1],body_z_rotation_contribution=groups[2],K_position_projected=float(kg[:3]@n),R_position_lateral=float(n@np.array(u['R'])[:3,:3]@n),R_rotation_diagonal=np.diag(u['R'])[3:].tolist(),fitness=float(ndt['fitness']),delta_bg_norm=float(np.linalg.norm(delta[9:12])),delta_ba_norm=float(np.linalg.norm(delta[12:15])),actual_delayed_update_ms=r['delayed_update_ms'],matched_latency_ms=r['after_available_ms'],replay_event_count_estimate=None)
  row['local_post_lateral_from_saved_increment']=row['prior_lateral']+row['correction_lateral_exact'];decomposition.append(row)
 for name,lo,hi in [('normal8',675,803),('right3',1317,1405)]:
  q=[v for v in decomposition if lo<=v['index']<=hi];z={k:stats([v[k] for v in q]) for k in ['prior_lateral','observation_lateral','local_post_lateral_from_saved_increment','correction_lateral_linear','correction_lateral_exact','position_measurement_contribution','roll_pitch_contribution','body_z_rotation_contribution','K_position_projected','R_position_lateral','fitness','delta_bg_norm','delta_ba_norm']}
  z['opposite_to_obs_translation_direction_count']=sum(v['correction_lateral_exact']*(v['observation_lateral']-v['prior_lateral'])<0 for v in q)
  z['descriptive_correlations_not_causal']={k:corr([v[k] for v in q],[v['prior_lateral'] for v in q]) for k in ['delta_ba_norm','fitness','R_position_lateral']}
  result['lateral'][name]=z
 result['checks'].update(max_saved_K_times_residual_delta_error=maxgainerr,max_saved_translation_innovation_vs_seed_difference=max_res_difference,algebra_only_no_filter_reexecution=True)
 # Clear per-frame values from public result; preserve non-reconstructive scalar decomposition privately.
 (out/'15d_lateral_decomposition.json').write_text(json.dumps(decomposition,indent=2))
 (out/'aggregate.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()))
 print(json.dumps(dict(far_debug=len(D),cov_nonzero=result['FAR']['input_pose_covariance_nonzero_count'],checks=result['checks']),indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','out','be','data']];main(p.parse_args())
