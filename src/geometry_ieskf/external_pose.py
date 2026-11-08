"""External map/base pose observation on the full 15D right-error state.
R is explicitly in the selected innovation chart, never inferred as calibrated.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import cho_factor,cho_solve
from .common import skew,jr,C
from .state import inject,chart_map
from .ndt_pose_observation import NdtPoseObservation,wrap
from .posterior import valid_prior
from ndt_global.shadow import se3

class ExternalPose:
 def __init__(self,lever,components=('x','y','z','yaw')):
  self.lever=np.asarray(lever);self.components=tuple(components)
  if len(set(components))!=len(components) or not components or any(k not in ('x','y','z','yaw','rx','ry','rz') for k in components):raise ValueError('COMPONENTS')
  if 'yaw' in components and any(k in components for k in ('rx','ry','rz')):raise ValueError('MIXED_ROTATION_CHART')
 def model(self,x,anchor=None):
  T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@self.lever;H=np.zeros((6,15));H[:3,:3]=np.eye(3);H[:3,6:9]=x.R@skew(self.lever);anchor=x.R if anchor is None else anchor;rv=Rotation.from_matrix(anchor.T@x.R).as_rotvec();H[3:,6:9]=np.linalg.inv(jr(rv));h=np.r_[T[:3,3],rv];indices={'x':0,'y':1,'z':2,'rx':3,'ry':4,'rz':5};values=[];rows=[]
  for key in self.components:
   if key=='yaw':z,J=NdtPoseObservation(self.lever).model(x);values.append(z[2]);rows.append(J[2])
   else:values.append(h[indices[key]]);rows.append(H[indices[key]])
  return np.array(values),np.array(rows)
 def residual(self,x,T):
  T=se3(T);h,H=self.model(x,x.R);rv=Rotation.from_matrix(x.R.T@T[:3,:3]).as_rotvec();z=dict(zip(('x','y','z'),T[:3,3]));z.update(dict(zip(('rx','ry','rz'),rv)));z['yaw']=np.arctan2(T[1,0],T[0,0]);r=np.array([z[k] for k in self.components])-h
  if 'yaw' in self.components:r[self.components.index('yaw')]=wrap(r[self.components.index('yaw')])
  return r,H
 def correct(self,x,P,T,R,projection=None):
  info=dict(accepted=False,covariance_calibrated=False,components=self.components)
  try:
   valid_prior(P);r,H=self.residual(x,T);R=np.asarray(R,float);m=len(r)
   if R.shape!=(m,m) or not np.isfinite(R).all() or not np.allclose(R,R.T,atol=1e-12):raise ValueError('INVALID_R')
   np.linalg.cholesky(R)
   if projection is not None:
    U=np.asarray(projection,float)
    if U.ndim!=2 or U.shape[1]!=m or np.linalg.matrix_rank(U)!=len(U):raise ValueError('INVALID_SUBSPACE')
    r=U@r;H=U@H;R=U@R@U.T
   S=H@P@H.T+R;K=cho_solve(cho_factor(S),H@P).T;d=K@r;info.update(innovation=r.tolist(),delta_proposed=d.tolist(),K=K.tolist(),conditional_NIS=float(r@cho_solve(cho_factor(S),r)),R=R.tolist())
   # Preserve prior bias/total-orientation validity limits, not optimizer position trust radius.
   if not np.isfinite(d).all():raise ValueError('NONFINITE')
   if np.linalg.norm(d[6:9])>C['max_total_rotation_rad']:raise ValueError('ATTITUDE_LINEARIZATION_DOMAIN')
   if np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s']:raise ValueError('BIAS_CHANGE_DOMAIN')
   if np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']:raise ValueError('BIAS_CHANGE_DOMAIN')
   A=np.eye(15)-K@H;B=chart_map(d);Q=B@(A@P@A.T+K@R@K.T)@B.T;Q=(Q+Q.T)/2;valid_prior(Q);y=inject(x,d)
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
  return y,Q,info|dict(accepted=True,reason='EXTERNAL_POSE_UPDATE',delta=d.tolist(),covariance_updates=1)
