"""Explicit policy selection around unchanged AR real CUDA/Deskew runner."""
import argparse,importlib.util,json,os,shlex,sys
from pathlib import Path
from geometry_ieskf.ndt_packet_observation import packet_covariance,InnovationGuardedPose
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--policy',choices=['legacy','single_use','guarded_single_use'],required=True);p.add_argument('--candidate-config',type=Path,default=Path(__file__).with_name('candidate.json'));p.add_argument('--window',default='lap');p.add_argument('--limit',type=int);a=p.parse_args();assert not a.private.exists(),'USE_A_NEW_PRIVATE_RUN_DIRECTORY';a.private.mkdir(parents=True);config=json.loads(a.candidate_config.read_text());spec=importlib.util.spec_from_file_location('ar_frozen',Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 if a.policy!='legacy':
  def covariance(ndt,components,prior):
   if a.policy=='guarded_single_use' and not ndt.get('av_integrity',{}).get('eligible',False):raise ValueError('AV_INTEGRITY:'+ndt.get('av_integrity',{}).get('reason','MISSING'))
   return packet_covariance(ndt['fitness'],components)
  m.pose_covariance=covariance
 if a.policy=='guarded_single_use':
  m.ExternalPose=lambda lever,components:InnovationGuardedPose(lever,components,config['nis_probability']);proxy=a.private/'matcher_proxy';command=[sys.executable,str(Path(__file__).with_name('matcher_integrity.py').resolve()),'--backend',str(a.matcher.resolve()),'--config',str(a.candidate_config.resolve()),'--audit',str((a.private/'integrity.jsonl').resolve())];proxy.write_text('#!/bin/sh\nexec '+shlex.join(command)+'\n');proxy.chmod(0o700);a.matcher=proxy.resolve()
 a.mode='F';a.stop_loss=None if a.policy=='legacy' else config['stop_after_no_observation_s'];m.main(a);result=json.loads(a.output.read_text());result.update(policy=a.policy,packet_R_calibrated=False,actual_causal_rematching=True,GNSS_used=False,production_default=False);a.output.write_text(json.dumps(result,indent=2)+'\n')
