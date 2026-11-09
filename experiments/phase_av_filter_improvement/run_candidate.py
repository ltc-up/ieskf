"""Explicit policy selection around unchanged AR real CUDA/Deskew runner."""
import argparse,importlib.util,json,os,shlex,sys
from pathlib import Path
from geometry_ieskf.robust_packet_pose import ScaleMixturePose
from geometry_ieskf.ndt_packet_observation import packet_covariance,InnovationGuardedPose,hypothesis_consensus_projection
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--policy',choices=['legacy','single_use','guarded_single_use','guarded_consensus_subspace','robust_consensus'],required=True);p.add_argument('--candidate-config',type=Path,default=Path(__file__).with_name('candidate.json'));p.add_argument('--history-seconds',type=float);p.add_argument('--window',default='lap');p.add_argument('--limit',type=int);a=p.parse_args();assert not a.private.exists(),'USE_A_NEW_PRIVATE_RUN_DIRECTORY';a.private.mkdir(parents=True);config=json.loads(a.candidate_config.read_text());spec=importlib.util.spec_from_file_location('ar_frozen',Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 if a.policy in ('guarded_consensus_subspace','robust_consensus'):
  assert config.get('ambiguity_policy')=='project_consensus','V2_CONFIG_REQUIRED'
  import types
  source=Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py';text=source.read_text();needle='Rv=pose_covariance(ndt,components,prior);projection=None';assert text.count(needle)==1;text=text.replace(needle,'Rv=pose_covariance(ndt,components,prior);projection=pose_projection(ndt,prior)');namespace={'__name__':'av_projection_runner'};exec(compile(text,str(source),'exec'),namespace);m=types.SimpleNamespace(**namespace)
  # Functions compiled above share namespace, so callback changes are explicitly
  # mirrored into it below rather than mutating the frozen source file.
  def projection(ndt,prior):
   if ndt['av_integrity']['reason']!='AMBIGUOUS_PARTIAL_PROJECTION_REQUIRED':return None
   U,meta=hypothesis_consensus_projection(prior.R,ndt['av_integrity']['hypotheses'],config['ambiguity_rotation_scale_m'],config['ambiguity_relative_rank_tolerance']);ndt['av_projection']=dict(matrix=U.tolist() if U is not None else None,anchor_time_ns=prior.t,**meta);return U
  namespace['pose_projection']=projection
 if a.policy!='legacy':
  def covariance(ndt,components,prior):
   if a.policy in ('guarded_single_use','guarded_consensus_subspace','robust_consensus') and not ndt.get('av_integrity',{}).get('eligible',False):raise ValueError('AV_INTEGRITY:'+ndt.get('av_integrity',{}).get('reason','MISSING'))
   return packet_covariance(ndt['fitness'],components)
  m.pose_covariance=covariance
 if a.policy in ('guarded_single_use','guarded_consensus_subspace','robust_consensus'):
  m.ExternalPose=(lambda lever,components:ScaleMixturePose(lever,components,config['mixture_quadrature_order'])) if a.policy=='robust_consensus' else (lambda lever,components:InnovationGuardedPose(lever,components,config['nis_probability']));proxy=a.private/'matcher_proxy';command=[sys.executable,str(Path(__file__).with_name('matcher_integrity.py').resolve()),'--backend',str(a.matcher.resolve()),'--config',str(a.candidate_config.resolve()),'--audit',str((a.private/'integrity.jsonl').resolve())];proxy.write_text('#!/bin/sh\nexec '+shlex.join(command)+'\n');proxy.chmod(0o700);a.matcher=proxy.resolve()
 if a.policy in ('guarded_consensus_subspace','robust_consensus'):namespace.update(pose_covariance=m.pose_covariance,ExternalPose=m.ExternalPose)
 if a.history_seconds is not None:
  assert .5<=a.history_seconds<=2.1,'RESEARCH_HISTORY_BOUND'
  original_replay=m.PoseDelayReplay
  class replay_with_history(original_replay):
   def __init__(self,*args,**kwargs):super().__init__(*args,**kwargs,horizon_ns=round(a.history_seconds*1e9))
  if a.policy in ('guarded_consensus_subspace','robust_consensus'):namespace['PoseDelayReplay']=replay_with_history
  else:m.PoseDelayReplay=replay_with_history
 a.mode='F';a.stop_loss=None if a.policy=='legacy' else config['stop_after_no_observation_s'];m.main(a);result=json.loads(a.output.read_text());result.update(policy=a.policy,packet_R_calibrated=False,actual_causal_rematching=True,GNSS_used=False,production_default=False,history_seconds=a.history_seconds if a.history_seconds is not None else .5);
 if a.policy in ('guarded_consensus_subspace','robust_consensus'):
  rows=json.loads((a.private/'transactions.json').read_text());result.pop('component_acceptance',None);ranks=[r['ndt'].get('av_projection',{}).get('observed_rank',6) for r in rows if r['update']['accepted']];result['accepted_observation_ranks']={str(k):ranks.count(k) for k in set(ranks)};result['full_component_acceptance']='NOT_ASSERTED_FOR_MIXED_SUBSPACE';health=json.loads((a.private/'health.json').read_text())
  for h,r in zip(health,rows):
   if r['update']['accepted'] and r['ndt'].get('av_projection',{}).get('observed_rank',6)<6:h['state']='PARTIAL_CONSTRAINT'
  (a.private/'health.json').write_text(json.dumps(health))
 a.output.write_text(json.dumps(result,indent=2)+'\n')
