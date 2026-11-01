"""Only AF generative experiments; no historical tests or external data."""
import argparse,time,numpy as np
from pathlib import Path
from geometry_ieskf.common import C,dump
from geometry_ieskf.partition import POSE
from geometry_ieskf.surface_observation import *
from synthetic_cases import *
def run():
 x,P=prior();out={'observability':[],'source_aliases':[],'density':[],'patch_partition':[]}
 for kind in ['horizontal','two','three','finite_local','finite_coplanar']:
  o=observation(kind);r,H,J,R=o.linearize(x);A=H[:,POSE];I=A.T@R.solve(A);out['observability'].append(dict(kind=kind,rank=int(np.linalg.matrix_rank(A)),singular_values=np.linalg.svd(A,compute_uv=False),information_eigenvalues=np.linalg.eigvalsh(I),null_X_Y_Yaw_norms=np.linalg.norm(A[:,[0,1,5]],axis=0)))
 o=observation('horizontal',9);r,H,J,R=o.linearize(x);b=linear_surface_update(P,H,r,R)
 for copies in [1,2,8,32]:
  ix=np.repeat(np.arange(len(o.points)),copies);a=SurfaceObservation(o.points[ix],o.sample_ids[ix],o.plane_ids[ix],o.planes,o.factor);rr,HH,JJ,RR=a.linearize(x);v=linear_surface_update(P,HH,rr,RR);out['source_aliases'].append(dict(copies=copies,canonical_samples=len(rr),information_difference_norm=float(np.linalg.norm(v['information']-b['information'])),posterior_difference_norm=float(np.linalg.norm(v['P_chart']-b['P_chart']))))
 for side in [3,5,9,17,33]:
  rr,HH,JJ,RR=observation('horizontal',side).linearize(x);v=linear_surface_update(P,HH,rr,RR);out['density'].append(dict(unique_samples=side**2,Z_variance=float(v['P_chart'][2,2]),Z_sigma=float(np.sqrt(v['P_chart'][2,2])),shared_plane_variance_floor=1/(1+1/.1**2)))
 for patches in [1,2,4,8]:
  parent=np.array(['plane0' for i in range(len(r))]);correct=linear_surface_update(P,H,r,SharedPlaneNoise(parent,J,o.planes,C['lidar_sigma_m']**2));np.testing.assert_allclose(correct['information'],b['information'],atol=1e-12);ids=np.array(['patch'+str(i%patches) for i in range(len(r))]);reg={key:Plane(key,o.planes['plane0'].normal,o.planes['plane0'].anchor,o.planes['plane0'].tangent,o.planes['plane0'].covariance) for key in np.unique(ids)};bad=SharedPlaneNoise(ids,J,reg,C['lidar_sigma_m']**2);v=linear_surface_update(P,H,r,bad);out['patch_partition'].append(dict(patches=patches,correct_shared_parent_Z_sigma=float(np.sqrt(b['P_chart'][2,2])),naive_independent_patch_Z_sigma=float(np.sqrt(v['P_chart'][2,2])),correct_shared_parent_information_difference=float(np.linalg.norm(correct['information']-b['information']))))
 o=observation('horizontal',3,True);r,H,J,R=o.linearize(x);rng=np.random.default_rng(20261101);S=o.planes['plane0'].covariance;draw=(rng.standard_normal((5000,3))@np.linalg.cholesky(S).T)@J.T+rng.standard_normal((5000,len(r)))*C['lidar_sigma_m'];out['known_correlated_law_MC_relative_error']=float(np.linalg.norm(np.cov(draw,rowvar=False)-R.dense())/np.linalg.norm(R.dense()));out['shared_error_law']=dict(tilt_units='rad',offset_units='m',covariance=S,generative_only=True)
 assert [v['rank'] for v in out['observability']]==[3,5,6,6,3]
 assert all(v['information_difference_norm']==0 for v in out['source_aliases'])
 assert all(v['Z_variance']>=v['shared_plane_variance_floor']-1e-12 for v in out['density'])
 out['synthetic_gate']='PASS';return out
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args();t=time.perf_counter();v=run();v['elapsed_s']=time.perf_counter()-t;dump(Path(a.output),v);print('AF generative gate PASS')
