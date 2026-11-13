"""Offline actual CUDA-native point provenance; independent diagnostic, no filter input."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from runtime import read_cloud
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--target',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
target=read_cloud(a.target).astype(float);tree=cKDTree(target);jobs=json.load(open(a.bindings));out=[]
for line in (a.run/'frames.jsonl').open():
 row=json.loads(line);i=row['index']
 if i%10 and i not in (333,739,883,1361):continue
 job=jobs[i];raw=np.fromfile(a.run/'deskew_ndt'/f"{row['ref']}_native.xyzi",'<f4').reshape(-1,4)[:,:3]
 b=next(b for b in row['branches'] if b['branch']=='ndt');T=np.asarray(b['ndt']['final']);t=np.asarray(b['prior']['v'])[:2];t=t/np.linalg.norm(t);U=np.array([[t[0],-t[1]],[t[1],t[0]]]);offset=0;parts={}
 for c in job['clouds']:
  n=c['width']*c['height'];source=raw[offset:offset+n];offset+=n;source=source[np.isfinite(source).all(axis=1)]
  source=source[np.linspace(0,len(source)-1,min(len(source),512),dtype=int)];world=source@T[:3,:3].T+T[:3,3];distance,nn=tree.query(world,workers=1);good=distance<2.;source=source[good];nn=nn[good]
  if len(nn):
   _,ids=tree.query(target[nn],k=20,workers=1);patch=target[ids];d=patch-patch.mean(axis=1)[:,None,:];eig,V=np.linalg.eigh(np.einsum('nki,nkj->nij',d,d)/20);normal=V[:,:,0];relative=source@T[:3,:3].T;J=np.c_[normal[:,:2]@U,np.cross(relative,normal)[:,2]/10.];L=J.T@J
  else:L=np.zeros((3,3))
  parts[c['frame']]=dict(information=L.tolist(),samples=int(len(nn)))
 assert offset==len(raw)
 L=sum((np.asarray(v['information']) for v in parts.values()),np.zeros((3,3)));v,V=np.linalg.eigh(L)
 out.append(dict(index=i,parts=parts,total=L.tolist(),eigenvalues=v.tolist(),weak=V[:,0].tolist(),sum_exact=True,interpretation='sampled original CUDA native point lineage, pre-voxel diagnostic; shared map errors are correlated'))
a.output.write_text(json.dumps(out))
