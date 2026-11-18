"""Additional saved-log checks only: repeats, timestamps, covariance floors,
and algebraic reference-time/current-time error accounting. No filter execution.
"""
import sys,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from analyze_existing import read,stats,WINDOWS,ORIGIN

def main(root,out,be):
 root=Path(root);out=Path(out);ref=np.load(Path(be)/'runtime/reference_lap.npz');D=[];lid=[];fs=[]
 for r in read(root/'far_recorded_init/records.jsonl'):
  if r['kind']=='timing_debug' and r['node']=='StateEstimation':D.append(r)
  elif r['kind']=='lidar':lid.append(r)
  elif r['kind']=='ekf':fs.append(r)
 comp=[r for r in read(root/'eskf15_recorded_init/states.jsonl') if r['cause']=='ndt_completion'];matches=[r for r in read(root/'eskf15_recorded_init/matches.jsonl') if 'ndt' in r];assert len(comp)==len(matches)
 result=dict(FAR_raw_pose_covariance_diagonal=np.diag(np.array(lid[0]['pose_cov']).reshape(6,6)).tolist(),windows={},checks={})
 result['checks']['saved_completion_pairs']=len(comp)
 for name,(lo,hi) in WINDOWS.items():
  d=[r for r in D if lo<=(r['sim_ns']-ORIGIN)*1e-9<=hi];run=[];lengths=[];last=None;Rcounts={}
  yawdiff=[];xyfloor=[];dlat=[];directreuse=0
  for r in d:
   v=r['values'];delay=v.get('pose/lidar_gicp/measured/msg_delay',0)
   if delay==last:run.append(1)
   else:
    if run:lengths.append(len(run))
    run=[1];last=delay
   for k in ['x_m','y_m','phi_rad','psi_rad']:
    z=v.get('kalman_filter/lidar_gicp/R_'+k);Rcounts.setdefault(k,{});key=str(round(z,10));Rcounts[k][key]=Rcounts[k].get(key,0)+1
   tau=v.get('pose/lidar_gicp/virtual/msg_delay',0);observed=v.get('kalman_filter/lidar_gicp/R_psi_rad',0);yawdiff.append(observed-max(.0002,.0001*(1+10*tau)))
   heading=v.get('pose/lidar_gicp/virtual/orientation_rad/yaw',0);n=np.array([-np.sin(heading),np.cos(heading),0]);diff=np.array([v.get('pose/lidar_gicp/virtual/position_m/'+k,0)-v.get('pose/lidar_gicp/measured/position_m/'+k,0) for k in ['x','y','z']]);dlat.append(n@diff)
  if run:lengths.append(len(run))
  z=dict(snapshot_count=len(d),constant_received_delay_run_lengths=stats(lengths),same_received_delay_adjacent_pairs=sum(max(0,n-1) for n in lengths),delay_run_count=len(lengths),R_value_counts=Rcounts,yaw_R_minus_floor_delay_formula=stats(yawdiff),virtual_minus_received_compensated_lateral=stats(dlat))
  sf=[r for r in fs if lo<=(r['stamp_ns']-ORIGIN)*1e-9<=hi];z['FAR_output_age_ms']=stats([(r['sim_ns']-r['stamp_ns'])/1e6 for r in sf]);z['FAR_body_vy_mps']=stats([r['velocity'][1] for r in sf]);result['windows'][name]=z
 # Algebraically separate saved reference-time correction and actual later state.
 frozen_tangent=np.load(Path(sys.argv[4])/'phase_as_accuracy_quality/single_lap/evaluation.npz')['tangent']
 decomp={r['index']:r for r in json.load(open(out/'15d_lateral_decomposition.json'))};lookup={int(t):i for i,t in enumerate(ref['header'])};detail=[];maxpair=0
 for m,st in zip(matches,comp):
  delta_t=st['received_wall_ns']-m['complete_wall_ns'];assert 0<=delta_t<5_000_000;maxpair=max(maxpair,delta_t)
  i=lookup.get(m['stamp_ns'])
  if i not in decomp or not m['update'].get('accepted'):continue
  prior=m['prior'];R=np.array(prior['R']);seed=np.array(m['seed']);lever=R.T@(np.array(prior['p'])-seed[:3,3]);inc=np.array(m['update']['delta']);post_ref=np.array(prior['p'])+inc[:3]-R@Rotation.from_rotvec(inc[6:9]).as_matrix()@lever
  base_prior=seed[:3,3];baseline=ref['G_0.075'][i];cur=st['stamp_ns'];right=np.searchsorted(ref['header'],cur);assert 0<right<len(ref['header']);left=right-1;f=(cur-ref['header'][left])/(ref['header'][right]-ref['header'][left]);gcur=(1-f)*ref['G_0.075'][left]+f*ref['G_0.075'][right]
  # Use scan-local tangent inferred from existing scalar residual and displacement.
  # The public audit loads the original tangent externally, not from scoring errors.
  tangent=frozen_tangent[i]
  if tangent is None:raise ValueError('REQUIRE_ORIGINAL_FROZEN_TANGENT')
  n=np.r_[-tangent[1],tangent[0],0.];ep=float(n@(base_prior-baseline));correction=float(n@(post_ref-base_prior));continuation=float(n@((np.array(st['position'])-post_ref)-(gcur-baseline)));actual=float(n@(np.array(st['position'])-gcur));assert abs(ep+correction+continuation-actual)<1e-10
  detail.append(dict(index=int(i),prior_lateral=ep,instant_update_lateral=correction,post_ref_lateral=ep+correction,repropagation_motion_minus_reference=continuation,actual_later_state_lateral=actual,reference_to_current_ms=(cur-m['stamp_ns'])/1e6,output_age_ms=(st['sim_ns']-cur)/1e6))
 result['checks']['max_completion_pair_lag_ms']=maxpair/1e6
 for name,lo,hi in [('normal8',675,803),('right3',1317,1405)]:
  q=[r for r in detail if lo<=r['index']<=hi];result['windows'][name]['15D_algebraic_later_state']={k:stats([r[k] for r in q]) for k in ['prior_lateral','instant_update_lateral','post_ref_lateral','repropagation_motion_minus_reference','actual_later_state_lateral','reference_to_current_ms','output_age_ms']}
 (out/'later_state_error_accounting.json').write_text(json.dumps(detail,indent=2));(out/'temporal_aggregate.json').write_text(json.dumps(result,indent=2));print('saved identity/repeat and exact additive accounting checked')
if __name__=='__main__':main(*sys.argv[1:4])
