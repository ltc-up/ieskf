"""Signed marginal-profile precision in physical directions, never separate R."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from scipy.spatial.transform import Rotation
from case_tools import *
from geometry_ieskf.information_audit import white_hessian
from geometry_ieskf.state import chart_map
from geometry_ieskf.partition import POSE
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
source,prior,P,factor,q,roots,ad=load_case(D);records=[]
for name,z in roots.items():
 o=CompactHuberMixture(prior,P,source,factor,q);d=decompose(o,z);H=d['H_white'];Q=o.L@cho_solve(cho_factor(H),o.L.T);G=chart_map(z);x=inject(prior,z);E=np.zeros((3,3));step=1e-6
 for j in range(3):
  up=Rotation.from_matrix(x.R@Rotation.from_rotvec(np.eye(3)[j]*step).as_matrix()).as_euler('xyz');dn=Rotation.from_matrix(x.R@Rotation.from_rotvec(-np.eye(3)[j]*step).as_matrix()).as_euler('xyz');E[:,j]=((up-dn+np.pi)%(2*np.pi)-np.pi)/(2*step)
 A=np.zeros((6,15));A[:3,:3]=np.eye(3);A[3:,6:9]=E@G[6:9,6:9];directions=[]
 for key,a in zip(['X','Y','Z','Roll','Pitch','Yaw'],A):
  aw=o.L.T@a;v=cho_solve(cho_factor(H),aw);v/=aw@v;contrib={k:float(v@M@v) for k,M in d['parts_white'].items()};total=sum(contrib.values());directions.append({'quantity':key,'units':'1/m^2' if key in ['X','Y','Z'] else '1/rad^2','profile_precision':total,'inverse_curvature_sigma':float(np.sqrt(a@Q@a)),'signed_terms':contrib,'signed_fractions':{k:x/total for k,x in contrib.items()},'white_profile_direction':v})
 Pp=P[np.ix_(POSE,POSE)];Lp=np.linalg.cholesky(Pp);Hgeom=o.Q.T@(H-np.eye(15))@o.Q;Hp=np.eye(6)+Lp.T@Hgeom[np.ix_(POSE,POSE)]@Lp;ev,V=np.linalg.eigh(Hp);loads=[]
 for j in range(6):
  v=Lp@V[:,j]/np.sqrt(np.diag(Pp));v/=np.linalg.norm(v);loads.append({'eigenvalue':float(ev[j]),'standardized_pose_axis_energy_loading':v*v})
 sweeps=[]
 for eps in [1e-4,1e-5,1e-6,1e-7]:
  Hfd,asym=white_hessian(o,z,eps);sweeps.append({'epsilon':eps,'relative_reconstruction_error':float(np.linalg.norm(H-Hfd)/np.linalg.norm(H)),'numeric_asymmetry_inf':asym})
 _,_,_,pairs=o.evaluate(z);near=float(np.min(abs(abs(pairs['residual'])-C['huber_delta_m'])))
 records.append({'root':name,'directions':directions,'pose_white_eigenvalues':ev,'pose_white_eigenvectors':V,'standardized_axis_loadings':loads,'hessian_stencil_sweep':sweeps,'nearest_Huber_breakpoint_residual_gap_m':near})
dump(O/'directional_information.json',records);print([(r['root'],r['hessian_stencil_sweep']) for r in records],flush=True)
