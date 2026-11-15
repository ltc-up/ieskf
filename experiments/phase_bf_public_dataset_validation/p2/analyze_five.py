"""Analyze the five completed registrations; never calls a matcher/filter."""
import argparse,csv,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def save(p,v):
 def clean(x):
  if isinstance(x,np.ndarray):return x.tolist()
  if isinstance(x,np.generic):return x.item()
  raise TypeError(type(x).__name__)
 p.write_text(json.dumps(v,default=clean,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def matrix(p):
 T=np.eye(4);T[:3,3]=p[:3];T[:3,:3]=Rotation.from_euler('XYZ',p[3:]).as_matrix();return T

def analyze(rec,trace,G,normalize):
 z=rec['result'];n=z['iterations'];assert len(trace)==n
 ds=np.array([t['delta'] for t in trace]);initial=np.ravel(z['parameters_internal'])-ds.sum(axis=0)
 poses=np.vstack([initial,initial+np.cumsum(ds,axis=0)]);transforms=np.array([matrix(p) for p in poses]);seed_error=np.max(abs(transforms[0]-np.array(z['seed_float'])));final_error=np.max(abs(transforms[-1]-z['final']));assert seed_error<1e-6 and np.max(abs(transforms[-1,:3,:3]-np.array(z["final"])[:3,:3]))<1e-6;assert np.array_equal(transforms[-1,:3,3].astype(np.float32),np.array(z["final"])[:3,3].astype(np.float32))
 u=np.r_[G[:2,0]/np.linalg.norm(G[:2,0]),0];d=np.array([-u[1],u[0],0]);records=[];history=[-trace[0]['score_before']];prev_eval=1;stable=0;directions=[];score_path=[trace[0]['score_before']];long_error=[u@(T[:3,3]-G[:3,3]) for T in transforms]
 for k,t in enumerate(trace):
  H=np.array(t['hessian']).reshape(6,6);assert np.isfinite(H).all() and np.max(abs(H-H.T))<1e-6
  delta=np.array(t['delta']);norm=np.linalg.norm(delta);assert abs(norm-t['accepted_step'])<1e-10
  unit=delta/norm;quad=float(unit@H@unit);flipped=quad>0;raw=t['raw_norm']*unit*(-1 if flipped else 1);grad=-H@raw
  rank=np.linalg.matrix_rank(H);assert rank==6
  normerr=abs(np.linalg.norm(grad)-t['gradient_norm'])/max(1,t['gradient_norm']);assert normerr<1e-6,('RECONSTRUCTED_GRADIENT',normerr)
  phi_direction=-float(grad@unit);assert phi_direction<0
  evaluations=t['evaluations']-prev_eval;prev_eval=t['evaluations'];ref=max(history[-3:]);rhs=ref+1e-4*t['accepted_step']*phi_direction;armijo=-t['score_after']<=rhs
  rel=abs(t['score_after']-t['score_before'])/max(abs(t['score_before']),1e-12);step_meets=norm<=.01;obj_meets=rel<=.001;stable=stable+1 if step_meets and obj_meets else 0
  history.append(-t['score_after']);score_path.append(t['score_after'])
  w=Rotation.from_matrix(transforms[k+1,:3,:3]@transforms[k,:3,:3].T).as_rotvec();scaled=np.r_[delta[:3],10*w];directions.append(scaled)
  geometry={L:normalize(H,poses[k,3:],u,L) for L in [5,10,20]}
  v=np.linalg.eigvalsh(-H)
  records.append(dict(iteration=k,pose_before_parameters=poses[k],pose_after_parameters=poses[k+1],T_before=transforms[k],T_after=transforms[k+1],logged_applied_delta=delta,
    newton_raw_reconstructed=raw,gradient_reconstructed=grad,reconstruction_relative_norm_error=normerr,descent_direction_reversed_from_raw=flipped,gradient_norm=t['gradient_norm'],
    raw_norm=t['raw_norm'],accepted_step=norm,initial_capped_step=max(.005,min(t['raw_norm'],.2)),score_before=t['score_before'],score_after=t['score_after'],relative_change=rel,
    source_count=z['source_n'],pairs_before=t['pairs_before'],pairs_after=t['pairs_after'],support_pair_delta=t['pairs_after']-t['pairs_before'],association_identity='UNKNOWN_NOT_LOGGED',
    evaluations_this_iteration=evaluations,backtracking_evaluations=evaluations-1,gll_reference_minimization=ref,gll_final_armijo_rhs=rhs,gll_final_armijo_satisfied=armijo,
    step_meets=step_meets,objective_meets=obj_meets,consecutive_stable=stable,termination_met=stable>=2,
    unscaled_negative_H_eigenvalues=v,unscaled_H_full_rank=rank,scaled_geometry=geometry,
    longitudinal_error_before=long_error[k],longitudinal_error_after=long_error[k+1],longitudinal_change=float(u@delta[:3])))
 assert stable>=2 if z['converged'] else stable<2
 cos=[];returns=[]
 for k in range(1,n):
  cos.append(float(directions[k]@directions[k-1]/np.linalg.norm(directions[k])/np.linalg.norm(directions[k-1])))
  a=directions[k-1];b=directions[k];returns.append(float(np.linalg.norm(a+b)/min(np.linalg.norm(a),np.linalg.norm(b))))
 # A sustained near-two-cycle requires 3 consecutive near returns and alternating score values.
 near=[]
 for k,(c,r) in enumerate(zip(cos,returns)):
  close_score=abs(score_path[k+2]-score_path[k])/max(abs(score_path[k]),1)<1e-4
  separated=abs(score_path[k+1]-score_path[k])/max(abs(score_path[k]),1)>1e-3
  if c<-.95 and r<.1 and close_score and separated:near.append(k+1)
 sustained=any(all(i+j in near for j in range(3)) for i in near)
 timing={k:z[k] for k in ['align_wall_ms','p2_pcl_fit_ms','quality_extra_ms','p2_prepare_ms','p2_target_read_ms','p2_region_ms','p2_source_read_ms','target_build_ms']}
 timing.update(service_ms=rec['service_ms'],encode_ms=rec['encode_ms'],decode_ms=rec['decode_ms'],write_ms=rec['result_write_ms'],align_minus_nested_pcl_ms=z['align_wall_ms']-z['p2_pcl_fit_ms'],ipc_serialization_other_ms=rec['service_ms']-z['p2_prepare_ms']-z['align_wall_ms']-z['quality_extra_ms'],imu_advance='NOT_EXECUTED',gpu_event_timers='UNKNOWN_NOT_EXPORTED')
 summary=dict(frame=rec['frame'],iterations=n,exit=z['exit'],all_history_equivalent=all(v['equal'] for v in rec['equivalence'].values()),max_history_numeric_difference=max(v.get('max_abs',0) for v in rec['equivalence'].values()),
    reconstruction_seed_transform_max_abs=seed_error,reconstruction_final_transform_max_abs=final_error,newton_gradient_max_relative_norm_error=max(r['reconstruction_relative_norm_error'] for r in records),
    longitudinal_seed_error=long_error[0],longitudinal_final_error=long_error[-1],longitudinal_optimization_change=long_error[-1]-long_error[0],
    score_start=score_path[0],score_end=score_path[-1],score_increase_fraction=(score_path[-1]-score_path[0])/score_path[0],score_decrease_iterations=[r['iteration'] for r in records if r['score_after']<r['score_before']],
    first_pairs=trace[0]['pairs_before'],last_pairs=trace[-1]['pairs_after'],source_count=z['source_n'],
    total_evaluations=trace[-1]['evaluations'],backtracking_iterations=[r['iteration'] for r in records if r['backtracking_evaluations']>0],total_backtracking_evaluations=sum(r['backtracking_evaluations'] for r in records),
    final_armijo_failed_iterations=[r['iteration'] for r in records if not r['gll_final_armijo_satisfied']],minimum_step_iterations=[r['iteration'] for r in records if abs(r['accepted_step']-.005)<1e-10],cap_step_iterations=[r['iteration'] for r in records if abs(r['accepted_step']-.2)<1e-10],
    scaled_direction_cosines=cos,scaled_two_step_return_ratios=returns,direction_cosine_below_minus_09_count=sum(x<-.9 for x in cos),sustained_near_two_cycle=sustained,two_cycle_thresholds=dict(L_m=10,cos=-.95,return_ratio=.1,score_period_rtol=1e-4,adjacent_score_rtol=1e-3,consecutive=3),
    last_step=records[-1]['accepted_step'],last_relative_change=records[-1]['relative_change'],last_step_meets=records[-1]['step_meets'],last_objective_meets=records[-1]['objective_meets'],final_stable_count=stable,
    initial_min_scaled_eigenvalue=records[0]['scaled_geometry'][10]['eigenvalues'][0],final_pre_step_min_scaled_eigenvalue=records[-1]['scaled_geometry'][10]['eigenvalues'][0],
    timing=timing,association_cycle='UNKNOWN_NO_ASSOCIATION_IDENTITIES',new_matches_in_analysis=0)
 return records,summary

def main(a):
 out=Path(a.out);out.mkdir(parents=True,exist_ok=False);run=Path(a.run);root=Path(a.root)
 receipt=json.loads((run/'execution_receipt.json').read_text());assert receipt['calls']==5 and receipt['completed']==[60,64,65,66,76]
 trace=[json.loads(l) for l in (run/'iterations.jsonl').open()]
 parent=Path(__file__).parents[1]
 def module(name,p):
  spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
 ad=module('bf_adapter',parent/'kitti_adapter.py');review=module('bf_review',parent/'review/analyze_existing.py')
 ref=np.load(root/'kitti/derived_time_aligned/offline_reference.npz');online=np.load(root/'kitti/derived_time_aligned/online.npz')
 frames={x['frame']:x for x in map(json.loads,(root/'results/kitti/B0_run1/frames.jsonl').open())}
 times=np.r_[online['scan_t'][0],[frames[i]['scan_epoch_ns'] for i in receipt['completed']]]
 G=ad.reference_at(ref['raw_t'],ref['raw_packets'],times)[1:];offset=0;summaries=[]
 for rec,g in zip(receipt['records'],G):
  n=rec['result']['iterations'];ts=trace[offset:offset+n];offset+=n;assert [x['iteration'] for x in ts]==list(range(n))
  records,summary=analyze(rec,ts,g,review.normalize_score);save(out/f"frame_{rec['frame']}_trajectory.json",records);summaries.append(summary)
 assert offset==len(trace)==119
 save(out/'summary.json',dict(baseline='6e43ecb0a53470f229e7d0882ccb4e7950ee14ec',registrations=5,trajectory_rows=len(trace),frames=summaries,no_fusion=True,no_parameter_changes=True,pose_reference='OXTS only offline analysis after matching; no GT Seed',checks='PASS_RECONSTRUCTION_EQUIVALENCE_KINEMATICS_CONVERGENCE'))
 print(json.dumps(summaries,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item() if isinstance(x,np.generic) else str(x),indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--root',required=True);p.add_argument('--out',required=True);main(p.parse_args())
