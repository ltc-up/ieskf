"""Frozen failedB Source/own-seed comparison only; never filter feedback."""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--target',required=True);p.add_argument('--ndt',required=True);p.add_argument('--gicp',required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',required=True);a=p.parse_args();tx=[json.loads(s) for s in (a.run/'transactions.jsonl').open()];ds=[json.loads(s) for s in (a.run/'deskew_meta_stream.jsonl').open()];a.private.mkdir(exist_ok=True);out=[];full=[]
for label,binary in [('NDT',a.ndt),('GICP',a.gicp)]:
 proc=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.private/(label+'_stderr.log')).open('w'),text=True)
 for i in [380,420,450,480]:
  r=tx[i];T=np.array(r['seed']);q=dict(case_id=str(i),header=r['ref'],group='DIAGNOSTIC_ONLY',scene='AX_B_FROZEN_FAILURE',target_path=a.target,target_sha='same_frozen_map0105',source_path=ds[i]['source_path'],seed_common=T.tolist(),q=50.,roi=[True,1,*T[:3,3],0,0,0,0,T[2,3]-10,T[2,3]+30,120,i+1]);proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();m=json.loads(proc.stdout.readline());full.append(dict(index=i,backend=label,request=q,result=m));Y=np.array(m['final']);out.append(dict(index=i,backend=label,converged=m['converged'],valid=m['candidate_valid'],iterations=m['iterations'],shift_m=float(np.linalg.norm(Y[:3,3]-T[:3,3])),Z_shift_m=float(Y[2,3]-T[2,3]),rotation_shift_deg=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(T[:3,:3].T@Y[:3,:3]).as_rotvec()))),fitness=m.get('fitness'),pairs=m.get('pairs',m.get('valid_pairs')),align_ms=m['align_wall_ms']))
 proc.stdin.close();proc.wait()
(a.private/'full_probes.json').write_text(json.dumps(full));Path(a.output).write_text(json.dumps(dict(probes=out,scope='FROZEN_SOURCE_DIAGNOSTIC_NOT_CLOSED_LOOP_NOT_TRUTH'),indent=2)+'\n');print(out)
