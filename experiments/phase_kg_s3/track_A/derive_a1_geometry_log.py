"""Derive explicit floor spectra and, only when identifiable, full chart R.

For all-floor full-rank A1, I_q=D^-2 independent of the unlogged eigenvectors.
This permits exact algebraic chart reconstruction from recorded poses. Other
cases are explicitly marked unavailable, never invented from eigenvalues.
"""
from log_io import open_text
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def derive(root,output):
 count=0;restored=0;unavailable=0;max_error=0;allfloor=0
 D=np.diag([.5]*3+[np.pi/60]*3)
 with Path(output).open('x') as out:
  for line in open_text(Path(root)/'frames.jsonl'):
   f=json.loads(line);u=f.get('update',{})
   raw=np.asarray(u.get('sandwich_variance_eigenvalues',[]),float)
   if not len(raw):continue
   count+=1;floored=np.maximum(raw,1);all_floor=bool((raw<=1).all());allfloor+=all_floor
   row=dict(ref_ns=f['ref_ns'],raw_normalized_variance_eigenvalues=raw.tolist(),floor_normalized_variance_eigenvalues=floored.tolist(),directions_floored=int((raw<=1).sum()),rank=len(raw),projected_R=u.get('R'),full_chart_R=None,reconstruction_available=False)
   if len(raw)==6 and u.get('retained_rank')==6 and all_floor:
    T=np.asarray(f['ndt']['final']);priorR=np.asarray(f['prior']['R']);q=np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_euler('XYZ')];L=np.zeros((6,6));h=1e-5
    def measurement(v):return np.r_[v[:3],Rotation.from_matrix(priorR.T@Rotation.from_euler('XYZ',v[3:]).as_matrix()).as_rotvec()]
    for k in range(6):
     d=np.eye(6)[k]*h;L[:,k]=(measurement(q+d)-measurement(q-d))/(2*h)
    R=L@D@D@L.T
    error=float(abs(np.linalg.eigvalsh(R)-np.linalg.eigvalsh(np.asarray(u['R']))).max())
    row.update(full_chart_R=R.tolist(),reconstruction_available=True,reconstruction_model='exact-full-rank-all-floor-identity',projected_R_spectrum_verification_max_abs=error)
    restored+=1;max_error=max(max_error,error)
   else:unavailable+=1
   out.write(json.dumps(row)+'\n')
 return dict(frames=count,all_directions_floored=allfloor,full_chart_R_exact_reconstructed=restored,full_chart_R_unavailable=unavailable,projected_R_spectrum_max_abs_error=max_error,uses_online_reference=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('output');a=p.parse_args();print(json.dumps(derive(a.root,a.output),indent=2))
