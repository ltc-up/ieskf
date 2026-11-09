"""CPU-only geometry plus existing actual NDT transactions; no feedback/new R."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
from geometry_ieskf.pose_uncertainty import native_pose_R,euler_xyz_to_body
from geometry_ieskf.state import State
from geometry_ieskf.common import jr
from ndt_global.integrity import MapSupport
from ndt_global.track_evaluation import offline_quality

def stat(x):
 x=np.asarray(x,float);x=x[np.isfinite(x)];return dict(zip(['min','p50','p95','max'],map(float,np.percentile(x,[0,50,95,100])))) if len(x) else None
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['run','target','evaluation','private','output']:p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();a.private.mkdir(parents=True,exist_ok=True);tx=json.loads((a.run/'transactions.json').read_text());ev=np.load(a.evaluation);support=MapSupport(np.fromfile(a.target,dtype='<f4',offset=8).reshape(-1,3));rows=[]
 for v in tx:
  T=np.asarray(v['ndt']['final']);src=np.fromfile(a.run/(str(v['ref'])+'_A1.bin'),dtype='<f4',offset=8).reshape(-1,3);g=support.diagnose(src,T);ge=g.get('geometry');info=np.zeros((6,6))
  if ge and ge['rank']:
   eig=np.array(ge['eigenvalues']);keep=eig>max(eig[-1]*1e-6,1e-12);V=np.asarray(ge['basis']).T;info=(V*eig[keep])@V.T
  R=T[:3,:3];long=np.r_[R[:,0],np.zeros(3)];lat=np.r_[R[:,1],np.zeros(3)];pose=State(np.asarray(v['prior']['p']),np.zeros(3),np.asarray(v['prior']['R']),np.zeros(3),np.zeros(3));native=native_pose_R(v['ndt'],('x','y','z','rx','ry','rz'),pose);D=np.zeros((6,6));D[:3,:3]=R.T;D[3:,3:]=10*jr(Rotation.from_matrix(pose.R.T@R).as_rotvec());Cn=D@native@D.T
  angles=np.asarray(v['ndt']['parameters_internal']).reshape(6)[3:];J=R@euler_xyz_to_body(angles);B=np.eye(6);B[3:,3:]=np.linalg.inv(J)/10;H=-B.T@np.asarray(v['ndt']['score_hessian'])@B;val=np.linalg.eigvalsh((H+H.T)/2)
  innovation=np.asarray(v['update'].get('innovation',[np.nan]*6));delta=np.asarray(v['update'].get('delta',[np.nan]*15));L=ge['eigenvalues'] if ge else [0]*6
  row=dict(index=v['index'],ref=v['ref'],fitness=v['ndt']['fitness'],iterations=v['ndt']['iterations'],converged=v['ndt']['converged'],valid=v['ndt']['candidate_valid'],support=g.get('nearby_fraction',0),support_cells=g.get('support_cells',0),unique_target=g.get('unique_target_points',0),planar_samples=g.get('planar_samples',0),covered_sectors=g.get('covered_sectors',0),rank=g.get('rank',0),geo_eigen_ratio=L[0]/max(L[-1],1e-20),normal_eigenvalues=ge['normal_eigenvalues'] if ge else [],geo_long=float(long@info@long),geo_lat=float(lat@info@lat),native_long_sigma=float(np.sqrt(Cn[0,0])),native_lat_sigma=float(np.sqrt(Cn[1,1])),native_Z_sigma=float(np.sqrt(native[2,2])),native_scaled_eigenvalues=np.linalg.eigvalsh(Cn).tolist(),score_H_SPD=bool(val[0]>0),score_H_eigen_ratio=float(val[0]/max(abs(val[-1]),1e-20)),scaled_gradient_norm=float(np.linalg.norm(B.T@np.asarray(v['ndt']['score_gradient']).reshape(6))),innovation_metric=float(np.linalg.norm(np.r_[innovation[:3],10*innovation[3:]])),translation_update=float(np.linalg.norm(delta[:3])),velocity_update=float(np.linalg.norm(delta[3:6])),bias_update=float(np.linalg.norm(delta[9:])),R_position_sigma=float(np.sqrt(np.asarray(v['update']['R'])[1,1])) if 'R' in v['update'] else None)
  row['geometry_info']=info.tolist();row['runtime_counts']={k:v['ndt'].get(k) for k in ['source_n','valid_pairs','candidate_pairs','inliers_at_float_pose','neighbor_points_at_float_pose','last_relative_objective_change','exit']}
  for k,direction in [('Z',np.array([0.,0.,1.,0,0,0])),('roll',np.r_[np.zeros(3),R[:,0]]),('pitch',np.r_[np.zeros(3),R[:,1]]),('yaw',np.r_[np.zeros(3),R[:,2]])]:row['geo_'+k]=float(direction@info@direction)
  rows.append(row)
  with (a.private/'quality.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  if len(rows)%200==0:print('CPU geometry',len(rows),flush=True)
 out=dict(frames=len(rows),geometry_scope='same real Source/final pose, CPU diagnostics; 10m rotation normalization',rank_counts={str(k):sum(v['rank']==k for v in rows) for k in range(7)},support=stat([v['support'] for v in rows]),geo_eigen_ratio=stat([v['geo_eigen_ratio'] for v in rows]),score_H_SPD_count=sum(v['score_H_SPD'] for v in rows),exact_association='join_quality.py uses original native timestamps',statistics_are_covariance=False)
 a.output.write_text(json.dumps(out,indent=2)+'\n')
