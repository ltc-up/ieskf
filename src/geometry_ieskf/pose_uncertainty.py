"""Coordinate-correct uncertainty candidates; no statistical calibration claim."""
import numpy as np
from scipy.spatial.transform import Rotation
from .common import jr
from .ndt_pose_observation import NdtPoseObservation, engineering_R
from .state import State

def euler_xyz_to_body(angles):
 a,b,c=np.asarray(angles);Ry=Rotation.from_rotvec([0,b,0]).as_matrix();Rz=Rotation.from_rotvec([0,0,c]).as_matrix()
 return np.column_stack((Rz.T@Ry.T@np.array([1.,0,0]),Rz.T@np.array([0.,1,0]),np.array([0.,0,1.])))

def native_pose_R(ndt,components,prior,T_map_pcm=None):
 """Native-only full covariance transport; never reads Fitness.

 T_map_pcm is an explicitly qualified fixed left frame transform. Default I is
 retained for the original same-frame API; real-data callers MUST independently
 qualify the frame/base contract, not infer it from this default.
 Input final/parameters/covariance all describe PCM-from-base, t+RxRyRz.
 """
 components=tuple(components)
 if not components or len(set(components))!=len(components) or any(k not in ('x','y','z','yaw','rx','ry','rz') for k in components) or ('yaw' in components and any(k in components for k in ('rx','ry','rz'))):
  raise ValueError('COMPONENTS')
 p=np.asarray(ndt['parameters_internal'],float).reshape(6)
 T=np.asarray(ndt['final'],float)
 if T.shape!=(4,4) or not np.isfinite(np.r_[p,T.ravel()]).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-12,rtol=0):
  raise ValueError('NATIVE_POSE')
 R=Rotation.from_euler('XYZ',p[3:]).as_matrix()
 if np.max(abs(R-T[:3,:3]))>2e-5 or not np.allclose(p[:3],T[:3,3],atol=2e-5,rtol=2e-7):
  raise ValueError('NATIVE_CHART_UNVERIFIED')
 A=np.eye(4) if T_map_pcm is None else np.asarray(T_map_pcm,float)
 if A.shape!=(4,4) or not np.isfinite(A).all() or not np.allclose(A[3],[0,0,0,1],atol=1e-12,rtol=0) or not np.allclose(A[:3,:3].T@A[:3,:3],np.eye(3),atol=1e-10,rtol=0) or not np.isclose(np.linalg.det(A[:3,:3]),1.,atol=1e-10,rtol=0):
  raise ValueError('MAP_PCM_CONTRACT')
 C=np.asarray(ndt['native_internal'],float)
 if C.shape!=(6,6) or not np.isfinite(C).all() or not np.allclose(C,C.T,atol=1e-10,rtol=0):
  raise ValueError('NATIVE_INVALID')
 np.linalg.cholesky(C)
 Rmap=A[:3,:3]@R
 J=euler_xyz_to_body(p[3:])
 M=np.zeros((len(components),6))
 rv=Rotation.from_matrix(prior.R.T@Rmap).as_rotvec()
 noise=np.linalg.solve(jr(rv),J)
 temp=State(np.zeros(3),np.zeros(3),Rmap,np.zeros(3),np.zeros(3))
 _,Hy=NdtPoseObservation([0,0,0]).model(temp)
 for i,k in enumerate(components):
  if k in ('x','y','z'):M[i,:3]=A[('x','y','z').index(k),:3]
  elif k=='yaw':M[i,3:]=Hy[2,6:9]@J
  else:M[i,3:]=noise[('rx','ry','rz').index(k)]
 out=M@C@M.T
 out=(out+out.T)/2
 np.linalg.cholesky(out)
 return out

def conditional_laplace_xy(H):
 H=np.asarray(H,float)
 if H.shape!=(6,6) or not np.isfinite(H).all() or not np.allclose(H,H.T,atol=1e-10):raise ValueError('HESSIAN_CONTRACT')
 A=-H[:2,:2];np.linalg.cholesky(A);return np.linalg.inv(A)

def source_floor_R(source_R,configured_variances):
 C=np.asarray(source_R);v=np.asarray(configured_variances)
 if C.shape!=(len(v),len(v)) or np.any(v<=0) or not np.isfinite(v).all() or not np.isfinite(C).all():raise ValueError('INVALID_SOURCE_NOISE')
 return np.diag(np.maximum(np.diag(C),v))
