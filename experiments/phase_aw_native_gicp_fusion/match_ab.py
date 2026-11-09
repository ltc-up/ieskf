"""Entire same-Source/same-float-seed lap, two actual persistent matchers."""
import argparse,json,subprocess,hashlib,time
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser()
for k in ['baseline','target','gicp','ndt','as-evaluation','output']:p.add_argument('--'+k,type=Path,required=True)
a=p.parse_args();assert not a.output.exists();a.output.mkdir(parents=True);tx=json.loads((a.baseline/'transactions.json').read_text());meta=[json.loads(v) for v in (a.baseline/'deskew_meta_stream.jsonl').read_text().splitlines()];corners=np.load(a.as_evaluation)['corner'];selected={int(np.flatnonzero(corners==k)[len(np.flatnonzero(corners==k))//2]):int(k) for k in [0,1,3,5,8,9]};sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();targetsha=sha(a.target);services={name:subprocess.Popen([str(binary.resolve())],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.output/(name+'_stderr.log')).open('w'),text=True) for name,binary in [('ndt',a.ndt),('gicp',a.gicp)]}
def call(name,q):
 proc=services[name];t=time.perf_counter_ns();proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();line=proc.stdout.readline();assert line,'SERVICE_TERMINATED:'+name;r=json.loads(line);r['protocol_wall_ms']=(time.perf_counter_ns()-t)*1e-6;return r
try:
 for i,(r,ds) in enumerate(zip(tx,meta)):
  source=Path(ds['source_path']);assert sha(source)==r['source_sha256'];seed=np.asarray(r['ndt']['seed_float']);v=seed[:3,3];q=dict(case_id=f'AW_AB_{i}',header=r['ref'],group=r['source_group'],scene='AW_frozen_same_source_seed',source_path=str(source.resolve()),target_path=str(a.target.resolve()),target_sha=targetsha,seed_common=seed.tolist(),roi=[True,1,*v,0,0,0,0,v[2]-10,v[2]+30,120,i+1],q=50.,curvature_probe=i in selected);N=call('ndt',q);G=call('gicp',q);row=dict(index=i,ref=r['ref'],corner=int(corners[i]),source_sha256=r['source_sha256'],seed=seed.tolist(),ndt=N,gicp=G)
  with (a.output/'matches.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  if i in selected:
   probes=[]
   for sign in [-1,1]:
    from scipy.spatial.transform import Rotation
    T=seed.copy();T[:3,3]+=sign*1.5*seed[:3,1];T[:3,:3]=T[:3,:3]@Rotation.from_rotvec([0,0,sign*np.deg2rad(.1)]).as_matrix();qq=q|dict(case_id=f'AW_PROBE_{i}_{sign}',seed_common=T.tolist());probes.append(dict(sign=sign,ndt=call('ndt',qq),gicp=call('gicp',qq)))
   repeat=call('gicp',q)
   with (a.output/'probes.jsonl').open('a') as f:f.write(json.dumps(dict(index=i,corner=selected[i],central_gicp=G,central_ndt=N,probes=probes,repeat_gicp=repeat))+'\n')
  if i%50==0:print(i+1,N['converged'],G['converged'],G.get('align_wall_ms'),flush=True)
finally:
 for proc in services.values():proc.stdin.close();proc.wait(timeout=30)
