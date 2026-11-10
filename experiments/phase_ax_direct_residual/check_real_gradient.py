import sys,json,subprocess,argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.direct_iterated import GeometryClient
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--binary',required=True);p.add_argument('--target',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=json.loads((a.run/'transactions.jsonl').read_text().splitlines()[0]);T=np.array(r['seed']);source=json.loads((a.run/'deskew_meta_stream.jsonl').read_text().splitlines()[0])['source_path'];q=dict(target_path=a.target,target_sha='test',source_path=source,seed_common=T.tolist(),shared_frame=True,roi=[False,1,0,0,0,0,0,0,0,-1e5,1e5,120,0]);proc=subprocess.Popen([a.binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);c=GeometryClient(proc);c.call(q);q['op']='evaluate';T[:3,:3]=T[:3,:3]@Rotation.from_rotvec([.03,-.01,.04]).as_matrix();q['seed_common']=T.tolist();mid=c.call(q);fd=[]
for k in range(6):
 e=1e-5;E=np.eye(4)
 if k<3:E[:3,:3]=Rotation.from_rotvec(np.eye(3)[k]*e).as_matrix()
 else:E[k-3,3]=e
 ep=c.call(q|dict(seed_common=(T@E).tolist()))['cost'];em=c.call(q|dict(seed_common=(T@np.linalg.inv(E)).tolist()))['cost'];fd.append((ep-em)/2/e)
proc.stdin.close();proc.wait();result=dict(analytic=np.array(mid['gradient']).ravel().tolist(),finite_difference=fd,max_error=float(np.max(abs(np.array(fd)-np.array(mid['gradient']).ravel()))));Path(a.output).write_text(json.dumps(result,indent=2));print(result)
