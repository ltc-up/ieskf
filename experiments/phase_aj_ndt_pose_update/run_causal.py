"""Autonomous offline C branch; restricted matcher is an external local service."""
import argparse,json,os,time,subprocess
from pathlib import Path
import numpy as np
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.replay import EventReplay
from geometry_ieskf.ndt_pose_observation import NdtPoseObservation,engineering_R,pose_xyyaw
from geometry_ieskf.geometry import GeometryFactor

class CausalNdtUpdate:
 def __init__(self,contract,service,template,out):
  self.factor=GeometryFactor(contract['extrinsic']['lever_body_imu_m']);self.obs=NdtPoseObservation(self.factor.lever);self.template=template;self.out=out;self.current=None;self.index=0;self.center=None;self.generation=0
  self.errors=(out/'ndt_service_stderr.log').open('w');self.proc=subprocess.Popen([service],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.errors,text=True);self.ledger=(out/'ndt_transactions.jsonl').open('w');(out/'sources').mkdir(exist_ok=True)
 def correct(self,prior,P,source,unused):
  begin=time.perf_counter();e=self.current;assert prior.t==e['header_ns'];T=self.factor.transform(prior);t=time.perf_counter();path=self.out/'sources'/f'{self.index:04d}.bin';np.array([len(source)],'<u8').tofile(path)
  with path.open('ab') as f:np.asarray(source,'<f4').tofile(f)
  prepare_ms=1000*(time.perf_counter()-t);j=dict(self.template);roi=list(j['roi']);rebuild=self.center is None
  # Same cylinder policy, now driven solely by this branch's current/past state.
  if self.center is None or np.linalg.norm(T[:3,3]-self.center)>=10.:
   self.center=T[:3,3].copy();self.generation+=1
  roi[2:5]=self.center.tolist();roi[9]=float(self.center[2]-10.);roi[10]=float(self.center[2]+30.);roi[12]=self.generation
  j.update(case_id=f'C_{self.index:04d}',header=e['header_ns'],group='C_NDT_XY_YAW',scene='AJ_frozen_window',source_path=str(path.resolve()),seed_common=T.tolist(),roi=roi)
  tick=time.perf_counter();self.proc.stdin.write(json.dumps(j)+'\n');self.proc.stdin.flush();line=self.proc.stdout.readline()
  if not line:raise RuntimeError('NDT_SERVICE_FAILED; preserve stderr and preceding transactions')
  ndt=json.loads(line);rpc_ms=1000*(time.perf_counter()-tick);Q=P.copy();x=prior.copy();info={'accepted':False,'reason':'NDT_NOT_CONVERGED','covariance_updates':0};rawT=np.array(ndt['final'])
  if ndt['converged'] and ndt['candidate_valid'] and np.isfinite(rawT).all():
   try:R=engineering_R(ndt['fitness'])
   except (ValueError,TypeError):info['reason']='INVALID_HISTORICAL_R'
   else:x,Q,info=self.obs.correct(prior,P,pose_xyyaw(rawT),R)
  if not info['accepted']:
   np.testing.assert_array_equal(Q,P)
   for key in ['p','v','R','bg','ba']:np.testing.assert_array_equal(getattr(x,key),getattr(prior,key))
  info.update(wall_ms=1000*(time.perf_counter()-begin),ndt=ndt,ndt_seed_origin='current_C_prediction_only',ndt_Z_minus_predicted_base_Z_m=float(rawT[2,3]-T[2,3]),source_prepare_ms=prepare_ms,ndt_rpc_ms=rpc_ms,target_build_included=rebuild,source_float32_sha256=sha(path),map_region_generation=self.generation,no_point_plane_update=True)
  self.ledger.write(json.dumps(dict(index=self.index,request=j,measurement=ndt,update=info))+'\n');self.ledger.flush();self.index+=1
  if self.index%25==0:print('processed',self.index,'last',info['reason'],flush=True)
  return x,Q,info
 def close(self):
  self.proc.stdin.close();self.proc.wait(timeout=30);self.errors.close();self.ledger.close()
  if self.proc.returncode:raise RuntimeError('NDT service exited nonzero')

def main(a):
 root=Path(a.data_root);out=root/'phase_aj_ndt_pose_update';data=json.load(open(root/'history/phase_v/results/input_continuous_extension.json'));contract=json.load(open(root/'history/phase_v/results/input_contract.json'));template=json.load(open(root/'phase_ah_deskew_z_ablation/ndt_jobs.json'))[0]
 target=root/'inputs/map_target.bin';assert sha(target)==contract['target_sha256'];template['target_path']=str(target.resolve());template['target_sha']=sha(target)
 baselineA=json.load(open(root/'history/phase_v/results/replay_inertial_wheel_control_continuous_extension.json'));baselineB=json.load(open(root/'phase_aa_z_validation/new_replay.json'));assert len(data['frames'])==284
 loaded=[];solver=CausalNdtUpdate(contract,a.ndt_service,template,out)
 def load(e):
  path=root/'history/phase_v'/e['source_file'];assert sha(path)==e['source_sha256'];solver.current=e;loaded.append(dict(source=e['source_sha256'],stamp=e['header_ns']));return np.load(path)
 replay=EventReplay(contract,source_loader=load);replay.solver=solver
 try:result=replay.run(data,None)
 finally:solver.close()
 result.update(no_ndt_input=False,observation_mode='NDT_XY_YAW_ONCE_NO_POINT_FACTORS',geometry_enabled=False,autonomous_match_seeds=True)
 for base in [baselineA,baselineB]:
  for k in ['state_time_ns','p_imu_map','v_imu_map','q_map_body_xyzw','bg_body_rad_s','ba_body_m_s2','P']:assert result['initialization'][k]==base['initialization'][k],k
  assert [r['header_ns'] for r in result['lidar_updates']]==[r['header_ns'] for r in base['lidar_updates']]
  assert [r['source_sha256'] for r in result['lidar_updates']]==[r['source_sha256'] for r in base['lidar_updates']]
 assert len(loaded)==len(data['frames']) and len(set(e['source'] for e in loaded))==284
 dump(out/'C_replay_private.json',result);dump(out/'input_identity_private.json',dict(input_stream_sha256=sha(root/'history/phase_v/results/input_continuous_extension.json'),baseline_A_sha256=sha(root/'history/phase_v/results/replay_inertial_wheel_control_continuous_extension.json'),baseline_B_sha256=sha(root/'phase_aa_z_validation/new_replay.json'),target_sha256=sha(target),loaded=loaded,all_initial_states_P_equal=True,all_source_times_equal=True));print('C complete',result['counts'],'sensor_s',result['elapsed_sensor_s'],'wall_s',result['cpu_replay_wall_s'],flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p.add_argument('--ndt-service',required=True);a=p.parse_args();assert a.data_root;main(a)
