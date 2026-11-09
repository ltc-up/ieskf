"""Persistent private matcher proxy: central + bounded hypotheses, one observation."""
import argparse,json,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.integrity import MapSupport
from geometry_ieskf.ndt_packet_observation import assess_integrity
p=argparse.ArgumentParser();p.add_argument('--backend',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);a=p.parse_args();config=json.loads(a.config.read_text());proc=subprocess.Popen([str(a.backend)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);support=None;target_identity=None

def match(request):
 proc.stdin.write(json.dumps(request)+'\n');proc.stdin.flush();line=proc.stdout.readline()
 if not line:raise RuntimeError('AV_MATCHER_BACKEND_TERMINATED')
 return json.loads(line)
try:
 for line in sys.stdin:
  q=json.loads(line);start=time.perf_counter_ns();center=match(q)
  if support is None or target_identity!=q['target_sha']:
   support=MapSupport(np.fromfile(q['target_path'],dtype='<f4',offset=8).reshape(-1,3));target_identity=q['target_sha']
  if center['source_n']==0:print(json.dumps(center),flush=True);continue
  source=np.fromfile(q['source_path'],dtype='<f4',offset=8).reshape(-1,3);geometry=support.diagnose(source,np.array(center['final']));hypotheses=[center]
  if center['converged'] and center['candidate_valid']:
   seed=np.array(q['seed_common']);scale=max(float(np.median(np.linalg.norm(source[:,:2],axis=1))),config['probe_translation_m']);angle=float(np.arctan(config['probe_translation_m']/scale))
   for sign in [-1,1]:
    T=seed.copy();T[:3,3]+=sign*config['probe_translation_m']*seed[:3,1];T[:3,:3]=Rotation.from_rotvec([0,0,sign*angle]).as_matrix()@seed[:3,:3];req=q|dict(case_id=q['case_id']+f'_probe{sign}',seed_common=T.tolist());hypotheses.append(match(req))
  verdict=assess_integrity(center,hypotheses,geometry,config);elapsed=(time.perf_counter_ns()-start)*1e-6;meta=dict(**verdict,geometry=geometry,hypotheses=[{k:h[k] for k in ['case_id','converged','candidate_valid','final','fitness','iterations','exit','align_wall_ms']} for h in hypotheses],transaction_ms=elapsed,hypotheses_are_independent=False)
  center['av_integrity']=meta
  with a.audit.open('a') as f:f.write(json.dumps(dict(header=q['header'],group=q['group'],**meta))+'\n')
  print(json.dumps(center),flush=True)
finally:
 proc.stdin.close();proc.wait(timeout=30)
