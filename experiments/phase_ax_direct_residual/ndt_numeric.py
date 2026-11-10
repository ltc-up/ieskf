"""Synthetic distribution counterexample + projected native-Gaussian interface."""
import argparse,json,subprocess,tempfile
from pathlib import Path
import numpy as np
from geometry_ieskf.direct_iterated import GeometryClient
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
with tempfile.TemporaryDirectory() as d:
 root=Path(d);pts=np.array([[1.5*i+.75,1.5*j+.75,.75] for i in range(-4,5) for j in range(-4,5)],'<f4');target=root/'target.bin';source=root/'source.bin'
 for f in [target,source]:
  with f.open('wb') as w:np.array([len(pts)],'<u8').tofile(w);pts.tofile(w)
 gaussian=root/'gaussian.bin'
 with gaussian.open('wb') as f:
  np.array([len(pts)],'<u8').tofile(f)
  for v in pts:
   k=np.floor(v/1.5).astype(int);code=sum(int(k[j]+2**20)<<(21*j) for j in range(3));np.array([code],'<u8').tofile(f);np.array([6],'<u4').tofile(f);np.r_[v,1,0,0,1,0,100].astype('<f4').tofile(f)
 proc=subprocess.Popen([a.binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);c=GeometryClient(proc);q=dict(target_path=str(target),target_sha='synthetic',source_path=str(source),gaussians=str(gaussian),seed_common=np.eye(4).tolist(),roi=[False,1,0,0,0,0,0,0,0,-100,100,120,0]);r=c.call(q);proc.stdin.close();proc.wait();H=np.array(r['H']);raw=np.array(r['raw_gicp_H']);out=dict(projected_rank=int(np.linalg.matrix_rank(H,tol=1e-6)),full_distribution_rank=int(np.linalg.matrix_rank(raw,tol=1e-6)),projected_tangential_information=float(np.linalg.norm(H[:,[2,3,4]])),raw_tangential_information=float(np.linalg.norm(raw[:,[2,3,4]])),full_distribution_qualification='FAILED_INFINITE_PLANE_OBSERVABILITY',projected_qualification='CONDITIONAL_GEOMETRY_ONLY',same_Gaussian_count=r['raw_gicp_pairs']);assert out['projected_rank']==3 and out['full_distribution_rank']==6;Path(a.output).write_text(json.dumps(out,indent=2)+'\n');print(out)
