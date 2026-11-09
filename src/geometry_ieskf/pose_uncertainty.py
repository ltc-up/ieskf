"""Coordinate-correct uncertainty candidates; no statistical calibration claim."""
import numpy as np
from scipy.spatial.transform import Rotation
from .common import jr
from .ndt_pose_observation import NdtPoseObservation, engineering_R
from .state import State

def euler_xyz_to_body(angles):
 a,b,c=np.asarray(angles);Ry=Rotation.from_rotvec([0,b,0]).as_matrix();Rz=Rotation.from_rotvec([0,0,c]).as_matrix()
 return np.column_stack((Rz.T@Ry.T@np.array([1.,0,0]),Rz.T@np.array([0.,1,0]),np.array([0.,0,1.])))

def native_pose_R(ndt,components,prior):
 engineering_R(ndt['fitness'])
 p=np.asarray(ndt['parameters_internal']).reshape(6);T=np.asarray(ndt['final']);R=Rotation.from_euler('XYZ',p[3:]).as_matrix()
 if np.max(abs(R-T[:3,:3]))>2e-5:raise ValueError('NATIVE_CHART_UNVERIFIED')
 C=np.asarray(ndt['native_internal'],float)
 if C.shape!=(6,6) or not np.isfinite(C).all() or not np.allclose(C,C.T,atol=1e-10):raise ValueError('NATIVE_INVALID')
 np.linalg.cholesky(C);J=euler_xyz_to_body(p[3:]);M=np.zeros((len(components),6));rv=Rotation.from_matrix(prior.R.T@R).as_rotvec();noise=np.linalg.inv(jr(rv))@J
 temp=State(np.zeros(3),np.zeros(3),R,np.zeros(3),np.zeros(3));_,Hy=NdtPoseObservation([0,0,0]).model(temp)
 for i,k in enumerate(components):
  if k in ('x','y','z'):M[i,('x','y','z').index(k)]=1
  elif k=='yaw':M[i,3:]=Hy[2,6:9]@J
  elif k in ('rx','ry','rz'):M[i,3:]=noise[('rx','ry','rz').index(k)]
  else:raise ValueError('COMPONENTS')
 out=M@C@M.T;out=(out+out.T)/2;np.linalg.cholesky(out);return out

def conditional_laplace_xy(H):
 H=np.asarray(H,float)
 if H.shape!=(6,6) or not np.isfinite(H).all() or not np.allclose(H,H.T,atol=1e-10):raise ValueError('HESSIAN_CONTRACT')
 A=-H[:2,:2];np.linalg.cholesky(A);return np.linalg.inv(A)

def source_floor_R(source_R,configured_variances):
 C=np.asarray(source_R);v=np.asarray(configured_variances)
 if C.shape!=(len(v),len(v)) or np.any(v<=0) or not np.isfinite(v).all() or not np.isfinite(C).all():raise ValueError('INVALID_SOURCE_NOISE')
 return np.diag(np.maximum(np.diag(C),v))
