"""Validate analytic removal of the known CUDA kernel regularizer vs raw points.
Only non-eigen-clamped Gaussians are reconstructible. No fitted scale/noise law.
"""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--gaussians',required=True);p.add_argument('--output',required=True);a=p.parse_args();pts=np.fromfile(a.target,'<f4',offset=8).reshape(-1,3);dt=np.dtype([('key','<u8'),('n','<u4'),('a','<f4',(9,))]);g=np.fromfile(a.gaussians,dt,offset=8);v=g['a'];M=np.zeros((len(g),3,3));M[:,0,0]=v[:,3];M[:,0,1]=M[:,1,0]=v[:,4];M[:,0,2]=M[:,2,0]=v[:,5];M[:,1,1]=v[:,6];M[:,1,2]=M[:,2,1]=v[:,7];M[:,2,2]=v[:,8];cov=np.linalg.inv(M);e=np.linalg.eigvalsh(cov);recover=e[:,0]>.01001*e[:,2];raw=cov*(g['n']/(g['n']-1))[:,None,None]-np.eye(3)/g['n'][:,None,None];raw_e=np.linalg.eigvalsh(raw);plane=recover&(raw_e[:,0]>=-1e-6)&(raw_e[:,1]>.02)&(raw_e[:,0]<.1*raw_e[:,1]);keys=np.floor(pts*np.float32(1/1.5)).astype(np.int64);code=((keys[:,0]+2**20).astype(np.uint64)|((keys[:,1]+2**20).astype(np.uint64)<<21)|((keys[:,2]+2**20).astype(np.uint64)<<42));order=np.argsort(code);unique,starts,counts=np.unique(code[order],return_index=True,return_counts=True);maxerr=0;count=0
for k in np.linspace(0,len(g)-1,256,dtype=int):
 if not recover[k]:continue
 j=np.searchsorted(unique,g['key'][k]);q=pts[order[starts[j]:starts[j]+counts[j]]].astype(float);assert len(q)==g['n'][k];center=q-q.mean(0);C=center.T@center/len(q);maxerr=max(maxerr,float(abs(C-raw[k]).max()));count+=1
out=dict(actual_CUDA_gaussians=len(g),nonclamped_reconstructible=int(recover.sum()),raw_surface_eligible=int(plane.sum()),checked_raw_voxels=count,covariance_max_abs_difference_m2=maxerr,qualification='NUMERIC_RAW_GEOMETRY_ONLY_NOT_SENSOR_COVARIANCE',clamped_case='REJECTED_NOT_UNREGULARIZED');assert maxerr<2e-5;Path(a.output).write_text(json.dumps(out,indent=2)+'\n');print(out)
