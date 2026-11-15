"""Read-only BF1 forensic statistics. No registration, filter replay or new update.
Only products of already recorded R/K/H/residual/delta/P and reference transforms.
"""
import argparse,csv,hashlib,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def scalar(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stat(x):
 x=np.asarray(x,float)
 return dict(n=len(x),mean=float(np.mean(x)),p50=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(np.max(x)),min=float(np.min(x)))
def yaw(R):return float(np.arctan2(R[1,0],R[0,0]))
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def skew(v):return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def basis(R):
 u=np.r_[R[:2,0]/np.linalg.norm(R[:2,0]),0.];d=np.array([-u[1],u[0],0.]);return u,d

def yaw_H(R):
 u=R[:,0];return np.array([-u[1],u[0],0.])/(u[0]**2+u[1]**2)@(-R@skew(np.array([1.,0,0])))

def normalize_score(H,angles,u,L):
 # Intrinsic Rx Ry Rz parameters -> world angular increment, holding z/roll/pitch nuisance fixed.
 rx,ry,rz=angles;Rx=Rotation.from_euler('x',rx).as_matrix();Ry=Rotation.from_euler('y',ry).as_matrix()
 J=np.column_stack(([1,0,0],Rx@[0,1,0],Rx@Ry@[0,0,1]));B=np.zeros((6,3));B[:2,:2]=np.column_stack((u[:2],[-u[1],u[0]]));B[3:,2]=np.linalg.solve(J,[0,0,1])/L
 A=B.T@(-(H+H.T)/2)@B;v,V=np.linalg.eigh(A)
 return dict(length_m=L,information=A,eigenvalues=v,weak_vector_s_d_Lyaw=V[:,0],weak_longitudinal_alignment=abs(V[0,0]),condition_positive=float(v[-1]/v[0]) if v[0]>0 else None)

def frozen_metrics(prior,candidate,G=None):
 P=np.asarray(prior['P']);R0=np.asarray(prior['R']);u,d=basis(R0);J=yaw_H(R0)
 F=np.zeros((3,6));F[0,:3]=u;F[1,:3]=d;F[2,3:]=J
 D=np.zeros((4,15));D[0,:3]=u;D[1,:3]=d;D[2,6:9]=J;D[3,5]=1
 R=np.asarray(candidate['R']);K=np.asarray(candidate['K']);r=np.asarray(candidate['residual']);delta=np.asarray(candidate['delta_proposed']);C=F@R@F.T
 projected_K=D@K;terms=projected_K*r[None,:]
 assert np.max(abs(terms.sum(axis=1)-D@delta))<1e-9
 inject=np.zeros((6,3));inject[:3,0]=u;inject[:3,1]=d;inject[3:,2]=R0.T@[0,0,1]
 gain=projected_K[:3]@inject;corr=C/np.sqrt(np.outer(np.diag(C),np.diag(C)))
 result=dict(variance_s_d_yaw=np.diag(C),correlation_s_d_yaw=corr,effective_self_gain_s_d_yaw=np.diag(gain),
  delta_s_d_yawlinear_vz=D@delta,signed_terms_by_innovation=terms,from_position_innovations=terms[:,:3].sum(axis=1),from_rotation_innovations=terms[:,3:].sum(axis=1),
  actual_yaw_candidate_change=wrap(yaw(R0@Rotation.from_rotvec(delta[6:9]).as_matrix())-yaw(R0)),
  candidate_committed=candidate['committed'],conditional_NIS=candidate['conditional_NIS'])
 if G is not None:
  U,V=basis(G[:3,:3]);e=np.asarray(prior['p'])-G[:3,3];ep=e+delta[:3]
  result.update(candidate_error_s_d=np.array([U@ep,V@ep]),prior_error_s_d=np.array([U@e,V@e]),
    candidate_yaw_error=wrap(yaw(R0@Rotation.from_rotvec(delta[6:9]).as_matrix())-yaw(G[:3,:3])))
 return result

def load(path):return [json.loads(x) for x in Path(path).open()]
def main(bf_root,be_cases,out):
 root=Path(bf_root);out=Path(out);out.mkdir(parents=True,exist_ok=False)
 input_paths=[root/'results/kitti'/run/name for run in ['B0_run1','B1_fixed_arrival','B1_run1'] for name in ['frames.jsonl','cycle_profile.jsonl']]
 input_paths += [root/'kitti/derived_time_aligned'/name for name in ['offline_reference.npz','online.npz','input_audit.json']]+[Path(be_cases)]
 before={str(p):digest(p) for p in input_paths}
 spec=importlib.util.spec_from_file_location('bf_adapter',Path(__file__).parents[1]/'kitti_adapter.py');ad=importlib.util.module_from_spec(spec);spec.loader.exec_module(ad)
 ref=np.load(root/'kitti/derived_time_aligned/offline_reference.npz');online=np.load(root/'kitti/derived_time_aligned/online.npz');audit=json.loads((root/'kitti/derived_time_aligned/input_audit.json').read_text())
 runs={k:load(root/'results/kitti'/run/'frames.jsonl') for k,run in [('B0','B0_run1'),('B1','B1_fixed_arrival')]}
 assert [r['available_time_ns'] for r in runs['B0']]==[r['available_time_ns'] for r in runs['B1']]
 times=np.array([r['scan_epoch_ns'] for r in runs['B0']],dtype=np.int64);pub=np.array([r['actual_published_state']['t_ns'] for r in runs['B0']],dtype=np.int64)
 G=ad.reference_at(ref['raw_t'],ref['raw_packets'],np.r_[online['scan_t'][0],times])[1:]
 GP=ad.reference_at(ref['raw_t'],ref['raw_packets'],np.r_[online['scan_t'][0],pub])[1:]
 coverage={v['frame']:v for v in audit['coverage']};details={};timings={};covdetail=[];phase_summary={}
 for mode,rows in runs.items():
  per=[]
  for row,g in zip(rows,G):
   n=row['ndt'];u,d=basis(g[:3,:3]);seed=np.asarray(row['seed']);final=np.asarray(n['final']);ep=seed[:3,3]-g[:3,3];ef=final[:3,3]-g[:3,3]
   sv={L:normalize_score(np.array(n['score_hessian']),np.array(n['parameters_internal']).ravel()[3:],u,L) for L in [5,10,20]}
   grad=np.asarray(n['score_gradient']).ravel();item=dict(frame=row['frame'],seed_error_s=float(u@ep),seed_error_d=float(d@ep),seed_xy_error=float(np.linalg.norm(ep[:2])),final_error_s=float(u@ef),final_error_d=float(d@ef),final_xy_error=float(np.linalg.norm(ef[:2])),
    coarse_reference_coverage=coverage[row['frame']]['fraction_within_1m'],inlier_fraction=n['inliers_at_float_pose']/n['source_n'],neighbors_fraction=n['neighbor_points_at_float_pose']/n['source_n'],
    support_pairs_per_source=n['valid_pairs']/n['source_n'],score_per_source=n['optimizer_score']/n['source_n'],score_per_valid_pair=n['optimizer_score']/n['valid_pairs'],
    valid_pair_fraction=n['valid_pairs']/n['candidate_pairs'],step=n['last_step_norm'],relative_change=n['last_relative_objective_change'],iterations=n['iterations'],exit=n['exit'],candidate_valid=n['candidate_valid'],
    final_step_meets=.01>=n['last_step_norm'],final_objective_meets=.001>=n['last_relative_objective_change'],normalized_geometry=sv,
    scaled_gradient_xyyaw_per_source=[float(u@np.r_[grad[:2],0]/n['source_n']),float(d@np.r_[grad[:2],0]/n['source_n'])])
   per.append(item)
  details[mode]=per
  profiles=load(root/'results/kitti'/('B0_run1' if mode=='B0' else 'B1_fixed_arrival')/'cycle_profile.jsonl');T=[]
  for r,p in zip(rows,profiles):
   align=r['ndt']['align_wall_ms'];quality=r['ndt']['quality_extra_ms'];serviceother=r['service_ms']-align-quality
   remainder=r['cycle_ms']-r['prepare_and_prior_ms']-r['input_decode_transform_ms']-r['service_ms']-r['rollback_replay_ms']
   t=dict(frame=r['frame'],imu_prior_composite_ms=r['prepare_and_prior_ms'],input_read_transform_write_ms=r['input_decode_transform_ms'],ndt_align_ms=align,quality_extra_ms=quality,service_load_ipc_serialize_residual_ms=serviceother,
    delayed_ingest_validate_ms=r['rollback_replay_ms'],unresolved_diagnostics_imu_to_availability_other_ms=remainder,record_serialization_write_ms=p['serialization_log_ms'])
   total=sum(v for k,v in t.items() if k!='frame');assert abs(total-p['instrumented_full_cycle_ms'])<1e-7
   assert min(v for k,v in t.items() if k!='frame')>=-1e-6
   t.update(full_ms=p['instrumented_full_cycle_ms'],simulated_epoch_to_available_ms=r['epoch_to_available_ms']);T.append(t)
  timing={k:stat([t[k] for t in T]) for k in T[0] if k!='frame'}
  timing.update(ndt_fraction_of_sum_ms=sum(t['ndt_align_ms'] for t in T)/sum(t['full_ms'] for t in T),
    all_frames_over50=sum(t['full_ms']>50 for t in T),excluding_initial_catchup_over50=sum(t['full_ms']>50 for t in T[1:]),
    converged_full_ms=stat([t['full_ms'] for t,r in zip(T,rows) if r['ndt']['converged']]),failed_full_ms=stat([t['full_ms'] for t,r in zip(T,rows) if not r['ndt']['converged']]),
    align_internal_ms_recorded_values=sorted(set(r['ndt']['align_internal_ms'] for r in rows)),target_build_ms_recorded_values=sorted(set(r['ndt']['target_build_ms'] for r in rows)))
  timings[mode]=timing
  with (out/f'{mode}_time_additive.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=T[0]);w.writeheader();w.writerows(T)
 # Exact error-energy decomposition on identical published epochs; no fitted alignment.
 errors={}
 for mode,rows in runs.items():
  E=[]
  for r,g in zip(rows,GP):
   s=r['actual_published_state'];u,d=basis(g[:3,:3]);err=np.asarray(s['p'])-g[:3,3]
   E.append([u@err,d@err,wrap(yaw(np.array(s['R']))-yaw(g[:3,:3]))])
  errors[mode]=np.asarray(E)
 phases=[('all57',20,76),('accepted46',20,65),('rejected11_prediction_tail',66,76)]
 for name,lo,hi in phases:
  mask=np.array([(lo<=r['frame']<=hi) for r in runs['B0']]);e0=errors['B0'][mask];e1=errors['B1'][mask]
  mse0=np.mean(e0**2,axis=0);mse1=np.mean(e1**2,axis=0);delta=mse1-mse0
  phase_summary[name]=dict(n=int(mask.sum()),B0_MSE_s_d_yaw=mse0,B1_MSE_s_d_yaw=mse1,Native_minus_Legacy_MSE_s_d_yaw=delta,
   delta_XY_MSE=float(delta[:2].sum()),Native_minus_Legacy_yaw_RMSE_deg=float(np.rad2deg(np.sqrt(mse1[2]))-np.rad2deg(np.sqrt(mse0[2]))))
 for row,g in zip(runs['B0'],G):
  both={k:frozen_metrics(row['prior'],row['frozen'][k],g) for k in ['B0','B1']}
  covdetail.append(dict(frame=row['frame'],common_valid=row['ndt']['converged'] and row['ndt']['candidate_valid'],B0=both['B0'],B1=both['B1']))
 covariance={}
 for name,lo,hi in [('early',20,35),('middle',36,55),('late_valid',56,65),('all_valid',20,65)]:
  subset=[v for v in covdetail if lo<=v['frame']<=hi];a=[s['B0'] for s in subset];b=[s['B1'] for s in subset]
  covariance[name]=dict(n=len(subset),median_native_to_legacy_variance_s_d_yaw=np.median([np.array(y['variance_s_d_yaw'])/np.array(x['variance_s_d_yaw']) for x,y in zip(a,b)],axis=0),
   median_B0_self_gain=np.median([x['effective_self_gain_s_d_yaw'] for x in a],axis=0),median_B1_self_gain=np.median([x['effective_self_gain_s_d_yaw'] for x in b],axis=0),
   median_abs_B0_yaw_from_translation=float(np.median([abs(x['from_position_innovations'][2]) for x in a])),median_abs_B1_yaw_from_translation=float(np.median([abs(x['from_position_innovations'][2]) for x in b])),
   frozen_candidate_lateral_RMSE_B0=float(np.sqrt(np.mean([x['candidate_error_s_d'][1]**2 for x in a]))),frozen_candidate_lateral_RMSE_B1=float(np.sqrt(np.mean([x['candidate_error_s_d'][1]**2 for x in b]))),
   frozen_candidate_yaw_RMSE_deg_B0=float(np.rad2deg(np.sqrt(np.mean([x['candidate_yaw_error']**2 for x in a])))),frozen_candidate_yaw_RMSE_deg_B1=float(np.rad2deg(np.sqrt(np.mean([x['candidate_yaw_error']**2 for x in b])))))
 # Outage kinematics from two saved states, not an IMU replay or new state.
 r0={m:runs[m][65-20]['actual_published_state'] for m in ['B0','B1']};r1={m:runs[m][-1]['actual_published_state'] for m in ['B0','B1']};u,d=basis(GP[65-20][:3,:3]);dt=(r1['B0']['t_ns']-r0['B0']['t_ns'])*1e-9
 dp0=np.asarray(r0['B1']['p'])-r0['B0']['p'];dp1=np.asarray(r1['B1']['p'])-r1['B0']['p'];dv0=np.asarray(r0['B1']['v'])-r0['B0']['v'];resid=dp1-dp0-dv0*dt
 outage=dict(dt_s=dt,initial_delta_p_s_d=[u@dp0,d@dp0],final_delta_p_s_d=[u@dp1,d@dp1],initial_delta_v_s_d=[u@dv0,d@dv0],delta_v_times_dt_s_d=[u@dv0*dt,d@dv0*dt],accumulated_acceleration_difference_residual_s_d=[u@resid,d@resid],
  initial_yaw_difference_deg=np.rad2deg(wrap(yaw(np.asarray(r0['B1']['R']))-yaw(np.asarray(r0['B0']['R'])))),final_yaw_difference_deg=np.rad2deg(wrap(yaw(np.asarray(r1['B1']['R']))-yaw(np.asarray(r1['B0']['R'])))),
  bg_difference_at65=np.asarray(r0['B1']['bg'])-r0['B0']['bg'],ba_difference_at65=np.asarray(r0['B1']['ba'])-r0['B0']['ba'])
 # Same existing paired conditional algebra on A2RL, never recompute a Kalman update.
 cases=load(be_cases);be=[x for x in cases if x['kind']=='BE'];comparison={};bedetail=[]
 for x in be:
  z={k:frozen_metrics(x['prior'],x[key]) for k,key in [('B0','Legacy_candidate'),('B1','Native_Raw_candidate')]};bedetail.append(dict(case_id=x['case_id'],selection_reasons=x['selection_reasons'],**z))
 def summarize_pairs(records):
  v={}
  for k in ['B0','B1']:
   v[k]=dict(abs_dvz=stat([abs(x[k]['delta_s_d_yawlinear_vz'][3]) for x in records]),abs_ds=stat([abs(x[k]['delta_s_d_yawlinear_vz'][0]) for x in records]),abs_dd=stat([abs(x[k]['delta_s_d_yawlinear_vz'][1]) for x in records]),
      abs_dyawlinear=stat([abs(x[k]['delta_s_d_yawlinear_vz'][2]) for x in records]),abs_yaw_from_position=stat([abs(x[k]['from_position_innovations'][2]) for x in records]))
  v['median_native_to_legacy_variance_s_d_yaw']=np.median([np.array(x['B1']['variance_s_d_yaw'])/x['B0']['variance_s_d_yaw'] for x in records],axis=0);return v
 comparison['A2RL_paired49']=summarize_pairs(bedetail);comparison['KITTI_valid46']=summarize_pairs([x for x in covdetail if x['common_valid']])
 summary=dict(baseline='49c3aad8d666deea3a9165117d09dd526df5360b',read_only=True,new_matches=0,replays=0,filter_updates=0,downloads=0,
     tail_geometry=details,timing=timings,error_decomposition=phase_summary,covariance_phases=covariance,outage=outage,A2RL_comparison=comparison,
     selected_frame_covariance=[x for x in covdetail if x['frame'] in [20,35,50,63,64,65]],checks=dict(identical_arrival_epochs=True,component_sum_timing=True,K_times_r_equals_recorded_delta=True))
 # Full cases stay private; summary contains no original pointcloud or full input pose.
 (out/'summary.json').write_text(json.dumps(summary,default=scalar,indent=2,allow_nan=False)+'\n')
 (out/'kitti_covariance_details.json').write_text(json.dumps(covdetail,default=scalar,indent=2,allow_nan=False)+'\n')
 (out/'a2rl_covariance_details.json').write_text(json.dumps(bedetail,default=scalar,indent=2,allow_nan=False)+'\n')
 after={str(p):digest(p) for p in input_paths};assert before==after
 (out/'input_identity.json').write_text(json.dumps(dict(before=before,after=after,unchanged=True),indent=2)+'\n')
 print('CHECKS PASS: existing inputs unchanged; no new matcher/filter/replay; additive times and recorded gain products verified')
 print(json.dumps(dict(error_decomposition=phase_summary,outage=outage,covariance_phases=covariance),default=scalar,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bf-root',required=True);p.add_argument('--be-cases',required=True);p.add_argument('--out',required=True);a=p.parse_args();main(a.bf_root,a.be_cases,a.out)
