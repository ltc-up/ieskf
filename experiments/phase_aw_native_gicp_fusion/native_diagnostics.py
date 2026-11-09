"""Existing Native implementation audit; covariance used diagnostically only."""
import argparse,json
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.pose_uncertainty import native_pose_R,euler_xyz_to_body
from geometry_ieskf.ndt_pose_observation import engineering_R
from ndt_global.matcher_geometry import ndt_native_information
from evaluate import stats
p=argparse.ArgumentParser();p.add_argument('--matches',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();rows=[json.loads(l) for l in a.matches.open()];values=[];floors=0;indef=0
for r in rows:
 n=r['ndt'];R=np.array(r['seed'])[:3,:3];prior=State(np.zeros(3),np.zeros(3),R,np.zeros(3),np.zeros(3));C=native_pose_R(n,('x','y','z','rx','ry','rz'),prior);e=np.r_[R[:,1],[0.,0.,0.]];eng=engineering_R(n['fitness']);D=np.sqrt(np.diag(C));cor=C/(D[:,None]*D[None,:]);I=n['kappa_at_float_pose']*np.linalg.inv(n['native_internal']);lam=np.linalg.eigvalsh(I);floors+=int(lam.min()<=1.0001e-6);A=np.zeros((6,6));A[:3,:3]=np.array(n['final'])[:3,:3];A[3:,3:]=np.linalg.inv(euler_xyz_to_body(np.array(n['parameters_internal']).reshape(6)[3:]))/10;S=A.T@(-np.array(n['score_hessian']))@A;indef+=int(np.linalg.eigvalsh(S).min()<=0);G=ndt_native_information(n);shape=float(np.sum(S*G)/(np.linalg.norm(S)*np.linalg.norm(G)));values.append(dict(native_lateral_std=float(np.sqrt(e@C@e)),fitness_lateral_std=float(np.sqrt(eng[0,0])),native_max_abs_correlation=float(np.max(abs(cor-np.eye(6)))),regularized_information_min_eigenvalue=float(lam.min()),score_vs_native_shape_cosine=shape,kappa=n['kappa_at_float_pose']))
a.output.write_text(json.dumps(dict(frames=len(rows),statistics={k:stats([v[k] for v in values]) for k in values[0]},regularization_floor_binding_frames=floors,normalized_negative_score_H_nonpositive=indef,existing_native_pose_R_reused=True,new_R_designed=False,score_H_is_not_native_GN_information=True),indent=2)+'\n')
