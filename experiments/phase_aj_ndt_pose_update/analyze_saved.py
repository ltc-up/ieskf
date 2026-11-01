"""Aggregate real branch statistics; no public raw trajectory or state payload."""
import argparse,os,json,collections,time
from pathlib import Path
import numpy as np
from geometry_ieskf.common import dump,sha
from geometry_ieskf.objective import state
from geometry_ieskf.ndt_pose_observation import NdtPoseObservation,engineering_R,pose_xyyaw
from geometry_ieskf.state import difference

def dist(v):
 a=np.asarray(v,dtype=float)
 if not a.size:return None
 return dict(zip(['P50','P95','P99','Max'],map(float,np.percentile(a,[50,95,99,100]))))
def covariance_summary(P):
 P=np.asarray(P);return dict(sigma_Z=float(np.sqrt(P[2,2])),sigma_vz=float(np.sqrt(P[5,5])),P_ZZ=float(P[2,2]),P_vzvz=float(P[5,5]),P_vzZ=float(P[5,2]),corr_Z_vz=float(P[5,2]/np.sqrt(P[2,2]*P[5,5])))
def main(root,public):
 root=Path(root);D=root/'phase_aj_ndt_pose_update';public=Path(public);paths=dict(A=root/'history/phase_v/results/replay_inertial_wheel_control_continuous_extension.json',B=root/'phase_aa_z_validation/new_replay.json',C=D/'C_replay_private.json');branches={k:json.load(open(p)) for k,p in paths.items()};C=branches['C'];l=C['lidar_updates'];contract=json.load(open(root/'history/phase_v/results/input_contract.json'));obs=NdtPoseObservation(contract['extrinsic']['lever_body_imu_m']);summaries={};binned={};t0=C['initialization']['state_time_ns']
 for key,b in branches.items():
  o=b['outputs'];xyz=np.array([s['p_imu_map'] for s in o]);vv=np.array([s['v_imu_map'] for s in o]);P=np.array([s['P'] for s in o]);eig=np.linalg.eigvalsh(P);start=np.array(b['initialization']['p_imu_map']);rows=b['lidar_updates'];actual=np.array([difference(state(r['corrected']),state(r['predicted'])) for r in rows]);accepted=np.array([r['accepted'] for r in rows]);times=np.array([(s['output_time_ns']-t0)*1e-9 for s in o]);Ts=np.array([r['header_ns'] for r in rows]);accTs=np.array([r['header_ns'] for r in rows if r['accepted']]);bound=np.r_[t0,accTs,o[-1]['output_time_ns']];gaps=np.diff(bound)*1e-9
  summaries[key]=dict(counts=b['counts'],sensor_duration_s=b['elapsed_sensor_s'],aborted=b['aborted'],output_count=len(o),output_interval_ns_counts=dict(collections.Counter(map(int,np.diff([s['output_time_ns'] for s in o])))),finite_state_covariance=bool(np.isfinite(np.r_[xyz.ravel(),vv.ravel(),P.ravel()]).all()),min_output_P_eigenvalue=float(eig.min()),max_output_P_asymmetry=float(abs(P-P.transpose(0,2,1)).max()),relative_Z_end_m=float(xyz[-1,2]-start[2]),relative_Z_range_m=float(np.ptp(xyz[:,2])),relative_Z_minmax_m=[float((xyz[:,2]-start[2]).min()),float((xyz[:,2]-start[2]).max())],vz_range_m_s=[float(vv[:,2].min()),float(vv[:,2].max())],vz_end_m_s=float(vv[-1,2]),max_abs_actual_delta_Z_m=float(abs(actual[:,2]).max()),max_abs_actual_delta_vz_m_s=float(abs(actual[:,5]).max()),abs_actual_delta_Z_m=dist(abs(actual[:,2])),abs_actual_delta_vz_m_s=dist(abs(actual[:,5])),max_actual_gyro_bias_delta_norm=float(np.linalg.norm(actual[:,9:12],axis=1).max()),max_actual_accel_bias_delta_norm=float(np.linalg.norm(actual[:,12:15],axis=1).max()),max_actual_rollpitch_error_norm_rad=float(np.linalg.norm(actual[:,6:8],axis=1).max()),height_covariance_initial=covariance_summary(b['initialization']['P']),height_covariance_final=covariance_summary(P[-1]),sigma_Z_max_m=float(np.sqrt(P[:,2,2]).max()),max_without_lidar_update_s=float(gaps.max()),CPU_wall_s=b['cpu_replay_wall_s'])
  groups=[]
  for second in range(15):
   take=(times>=second)&(times<second+1)
   if take.any():groups.append(dict(elapsed_bin_s=[second,second+1],relative_Z_m_median=float(np.median(xyz[take,2]-start[2])),relative_Z_m_P10_P90=np.percentile(xyz[take,2]-start[2],[10,90]).tolist(),vz_m_s_median=float(np.median(vv[take,2])),vz_m_s_P10_P90=np.percentile(vv[take,2],[10,90]).tolist(),sigma_Z_m_median=float(np.median(np.sqrt(P[take,2,2])))))
  binned[key]=groups
 # Post-hoc saved-prior matrix audit; not another recursive or NDT replay.
 audit=[];errstate=[];errP=[];obs_times=[]
 for i,r in enumerate(l):
  if r['reason'] in ['NDT_NOT_CONVERGED','INVALID_HISTORICAL_R']:continue
  x=state(r['predicted']);P=np.array(r['predicted']['P']);ndt=r['ndt'];z=pose_xyyaw(np.array(ndt['final']));R=engineering_R(ndt['fitness']);begin=time.perf_counter();y,Q,info=obs.correct(x,P,z,R);ms=1000*(time.perf_counter()-begin);obs_times.append(ms);assert info['accepted']==r['accepted'] and info['reason']==r['reason'];delta=difference(y,state(r['corrected']));errstate.append(float(abs(delta).max()));errP.append(float(abs(Q-np.array(r['corrected']['P'])).max()));audit.append(dict(index=i,reason=info['reason'],wall_ms=ms,max_state_error=errstate[-1],max_P_error=errP[-1]))
 assert max(errstate)<1e-10 and max(errP)<1e-10
 dump(D/'matrix_audit_private.json',audit)
 accepted=[r for r in l if r['accepted']];observed=[r for r in l if 'innovation' in r];innovation=np.array([r['innovation'] for r in observed]);innovacc=np.array([r['innovation'] for r in accepted]);accidx=[i for i,r in enumerate(l) if r['accepted']];recovery=[i for i in accidx if i==0 or not l[i-1]['accepted']];longest=0;cur=0
 for r in l:
  cur=0 if r['accepted'] else cur+1;longest=max(cur,longest)
 # Actual gain decomposition retains signed contributions from all three coordinates.
 contrib=[]
 for i,r in enumerate(l):
  if 'K' not in r:continue
  components=np.array(r['K'])[5]*np.array(r['innovation']);d=float(np.sum(components));assert abs(d-r['delta_proposed'][5])<1e-12;contrib.append(dict(index=i,accepted=r['accepted'],vz_from_XY_yaw=components.tolist(),sum=d))
 dump(D/'vz_contributions_private.json',contrib)
 def finite_num(v):return v is not None and np.isfinite(v)
 def ndtvalues(key,converged=None):return [r['ndt'][key] for r in l if finite_num(r['ndt'].get(key)) and (converged is None or r['ndt']['converged']==converged)]
 total=np.array([r['wall_ms'] for r in l])
 # Source load quantiles remain in Diagnostics; continuous total includes all other work.
 report=dict(branches=summaries,actual_C_reasons=dict(collections.Counter(r['reason'] for r in l)),NDT_exits=dict(collections.Counter(r['ndt']['exit'] for r in l)),engineering_guard_failures=dict(collections.Counter(g for r in l for g in r.get('guard_failures',[]))),C_accepted=66 if len(accepted)==66 else len(accepted),C_lidar_retention_fraction=len(accepted)/len(l),C_recovery_events=len(recovery),C_longest_rejection_streak_frames=longest,C_first_accept_elapsed_s=(l[accidx[0]]['header_ns']-t0)*1e-9 if accidx else None,C_last_accept_elapsed_s=(l[accidx[-1]]['header_ns']-t0)*1e-9 if accidx else None,C_recovery_elapsed_s=[(l[i]['header_ns']-t0)*1e-9 for i in recovery],innovation_abs_XY_yaw_all_components=[dist(abs(innovation[:,k])) for k in range(3)],innovation_abs_XY_yaw_accepted_components=[dist(abs(innovacc[:,k])) for k in range(3)],NDT_Z_minus_predicted_base_Z_abs_m=dist([abs(r['ndt_Z_minus_predicted_base_Z_m']) for r in l]),NDT_fitness_finite=dist(ndtvalues('fitness')),NDT_fitness_converged=dist(ndtvalues('fitness',True)),NDT_iterations=dist(ndtvalues('iterations')),NDT_align_ms=dist(ndtvalues('align_wall_ms')),NDT_rpc_including_extra_quality_ms=dist([r['ndt_rpc_ms'] for r in l]),lidar_callback_ms=dist(total),lidar_callback_above_50ms=int(sum(total>50)),NDT_target_initial_build_ms=l[0]['ndt']['target_build_ms'],NDT_extra_quality_ms=dist(ndtvalues('quality_extra_ms')),source_prepare_ms=dist([r['source_prepare_ms'] for r in l]),matrix_audit_count=len(audit),matrix_audit_max_state_error=max(errstate),matrix_audit_max_P_error=max(errP),ESKF_frozen_posthoc_update_ms=dist(obs_times),matrix_audit_scope='Saved priors only, no NDT calls or state recursion; separate timing not retained during the continuous run',C_diagnostics={**C['diagnostics'],'gpu':'external unchanged CUDA-NDT service; GPU memory unmeasured'},C_no_point_factors=True,real_C_state_P_commits=len(accepted),production_published_states=0,source_loading_same_for_all=True,initial_state_P_same_for_all=True)
 dump(D/'aggregate_summary.json',report);dump(D/'binned_summary.json',binned);dump(public/'key_results.json',report);dump(public/'height_binned_summary.json',binned);print(json.dumps({k:v for k,v in report.items() if k not in ['branches','C_diagnostics']},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p.add_argument('--public-output',required=True);a=p.parse_args();main(a.data_root,a.public_output)
