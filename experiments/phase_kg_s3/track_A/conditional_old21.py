"""Supplemental historical Source/model-prior diagnostic; never continuous truth.

Only the approved prior and fresh original Align initialize MAP. Saved old
candidates are evaluated AFTER the two-start result and cannot select a branch.
The KG-S2 conditional zeroBias/defaultP/model-velocity prior is reused exactly;
this is not a recovered mature historical ESKF prior.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
REPO=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(REPO/'src'),str(REPO/'experiments/phase_be_e0_e4')]
from runtime import Service
from geometry_ieskf.state import State
from score_map import FrozenMap,prior_identity,state_q
from strategy import factory
from realtime_runner import preflight_and_snapshot

def run(config_file):
 cfg=json.loads(Path(config_file).read_text());out=Path(cfg['output'])
 old=Path(cfg['kg_s2_conditional_input']);requests_path=Path(cfg['historical_requests'])
 bundle=json.loads(old.read_text());record=bundle['series'][0]['records'][0]
 requests=json.loads(requests_path.read_text());request=requests[21]
 if record['t']!=request['header']:raise ValueError('OLD21_TIMESTAMP_IDENTITY')
 actual=hashlib.sha256(Path(record['source']).read_bytes()).hexdigest()
 if actual!=record['source_sha'] or actual!=hashlib.sha256(Path(request['source_path']).read_bytes()).hexdigest():raise ValueError('OLD21_SOURCE_IDENTITY')
 actual_target=hashlib.sha256(Path(request['target_path']).read_bytes()).hexdigest()
 if actual_target!=request['target_sha']:raise ValueError('OLD21_TARGET_IDENTITY')
 cfg['paths']['target']=request['target_path'];cfg['target_sha']=actual_target
 identity=preflight_and_snapshot(cfg,out);(out/'config.frozen.json').write_text(json.dumps(cfg,indent=2))
 lever=np.asarray(bundle['T_base_imu'])[:3,3];p=record['prior']
 x=State(np.asarray(p['p']),np.asarray(p['v']),np.asarray(p['R']),np.asarray(p['bg']),np.asarray(p['ba']),int(record['t']));P=np.asarray(p['P'])
 original=Service(cfg['paths']['ndt'],out/'original_ndt.stderr');strategy=factory(cfg,cfg['paths'],out,lever)
 begin=time.monotonic_ns()
 try:
  # Preserve the actual current original NDT model and frozen iteration policy.
  align_request=dict(request,q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
  align,align_ms=original.call(align_request)
  observer=strategy.create_observer(record['source'],actual,actual_target,request['roi'],prior_identity(x,P),align)
  y,Q,info=observer.correct(x,P,np.asarray(align['final']),None)
  # No probe candidate is supplied to the optimizer or candidate selector.
  frozen=FrozenMap(x,P,lever,observer.evaluate,observer.contract)
  probe_T=np.asarray(request['identity']['old_final']);probe_z=frozen.align_start(probe_T)
  try:
   probe=frozen.value(probe_z);probe={k:v for k,v in probe.items() if k not in ('Hq','J','H','g')};probe['qualification']='POSTHOC_SAVED_NDT_NEAR_CANDIDATE_ONLY_NOT_INITIALIZATION'
  except ValueError as exc:probe=dict(status=str(exc),qualification='POSTHOC_ONLY')
  ybase=y.p-y.R@lever
  lateral=float((np.asarray(record['seed'])[:3,:3].T@(ybase-np.asarray(record['seed'])[:3,3]))[1])
  comparisons=[]
  for candidate in info.get('candidates',[]):
   if candidate.get('failure'):continue
   z=np.asarray(candidate['z']);value=frozen.value(z);raw=observer.evaluate(value['q'])
   original_H=np.asarray(raw['original']['hessian']);corrected_H=np.asarray(raw['corrected']['hessian'])
   old_MAP_H=value['H']-observer.contract.beta*value['J'].T@(original_H-corrected_H)@value['J']
   comparisons.append(dict(label=candidate['label'],pose_q=value['q'],original=raw['original'],corrected=raw['corrected'],score_gradient_identical=raw['original']['score']==raw['corrected']['score'] and raw['original']['gradient']==raw['corrected']['gradient'],original_score_newton_direction=np.linalg.lstsq(original_H,-np.asarray(raw['gradient']),rcond=1e-12)[0],corrected_score_newton_direction=np.linalg.lstsq(corrected_H,-np.asarray(raw['gradient']),rcond=1e-12)[0],original_MAP_newton_direction=np.linalg.lstsq(old_MAP_H,-value['g'],rcond=1e-12)[0],corrected_MAP_newton_direction=np.linalg.lstsq(value['H'],-value['g'],rcond=1e-12)[0]))
  result=dict(qualification='KG-S2 model conditional zeroBias/defaultP/model velocity; not mature historical prior; not continuous or accuracy certification',source_sha256=actual,target_sha256=actual_target,record_sha256=hashlib.sha256(json.dumps(record,sort_keys=True).encode()).hexdigest(),conditional_bundle_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),requests_sha256=hashlib.sha256(requests_path.read_bytes()).hexdigest(),prior_hash=prior_identity(x,P),prior=record['prior'],header_ns=record['t'],source_n=record['source_n'],target_n=request['identity']['target_n'],original_align=align,original_align_ms=align_ms,map_update=info,posterior=dict(p=y.p,v=y.v,R=y.R,bg=y.bg,ba=y.ba,P=Q),lateral_delta_in_original_seed_body_m=lateral,distance_to_saved_near_candidate_m=float(np.linalg.norm(ybase-probe_T[:3,3])),saved_near_posthoc_probe=probe,posthoc_original_corrected_comparisons=comparisons,posthoc_comparisons_used_for_selection=False,same_frame_saved_wrong_4_to_5m_candidate='UNKNOWN_NOT_FOUND_NOT_SYNTHESIZED',online_evaluation_reference_used=False,total_wall_ms=(time.monotonic_ns()-begin)/1e6)
  def clean(value):
   if isinstance(value,np.ndarray):return value.tolist()
   if isinstance(value,np.generic):return value.item()
   raise TypeError(type(value).__name__)
  (out/'result.json').write_text(json.dumps(result,indent=2,default=clean)+'\n')
  print(json.dumps({k:result[k] for k in ['qualification','source_n','lateral_delta_in_original_seed_body_m','distance_to_saved_near_candidate_m','same_frame_saved_wrong_4_to_5m_candidate','total_wall_ms']},indent=2))
 finally:original.close();strategy.close()
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('config');run(p.parse_args().config)
