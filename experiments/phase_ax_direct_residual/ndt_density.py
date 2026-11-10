"""Real original CUDA builder, exact repeated synthetic planar map records."""
import argparse,subprocess,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--exporter',required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',required=True);a=p.parse_args();a.private.mkdir(exist_ok=True);pts=np.array([[x,y,.5] for x in np.linspace(.15,1.35,7) for y in np.linspace(.15,1.35,7)],'<f4');dt=np.dtype([('key','<u8'),('n','<u4'),('a','<f4',(9,))]);out=[]
for copies in [1,10]:
 point=a.private/f'plane_{copies}.bin';gpath=a.private/f'gaussian_{copies}.bin'
 with point.open('wb') as f:np.array([len(pts)*copies],'<u8').tofile(f);np.repeat(pts,copies,axis=0).tofile(f)
 result=subprocess.run([a.exporter,str(point),str(gpath)],capture_output=True,text=True,check=True);r=np.fromfile(gpath,dt,offset=8);v=r[0]['a'];M=np.array([[v[3],v[4],v[5]],[v[4],v[6],v[7]],[v[5],v[7],v[8]]]);e=np.linalg.eigvalsh(M);out.append(dict(exact_copies=copies,stored_records=int(r[0]['n']),mean=v[:3].tolist(),inverse_covariance_eigenvalues=e.tolist(),normal_anisotropy=float(e[2]/e[1]),eligible=bool(e[2]>10*e[1]),actual_CUDA_builder=json.loads(result.stdout)['actual_CUDA_builder']))
Path(a.output).write_text(json.dumps(dict(cases=out,qualification='FAILED_SAMPLING_INVARIANCE_NATIVE_GAUSSIAN_SCALE_OR_ELIGIBILITY'),indent=2)+'\n');print(out)
