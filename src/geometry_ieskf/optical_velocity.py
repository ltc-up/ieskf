"""Actual optical sensor axes, with body/IMU lever and full15D covariance.
Engineering covariance is supplied explicitly; no zero-lateral assumption.
"""
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from .common import skew,C
from .state import inject,chart_map
from .posterior import valid_prior
class OpticalVelocity:
 def __init__(self,imu_lever,T_body_sensor,axes=(1,)):
  self.T=np.asarray(T_body_sensor,float);self.axes=tuple(axes)
  if self.T.shape!=(4,4) or not np.isfinite(self.T).all() or not np.allclose(self.T[3],[0,0,0,1]) or not np.allclose(self.T[:3,:3].T@self.T[:3,:3],np.eye(3),atol=1e-8) or not np.isclose(np.linalg.det(self.T[:3,:3]),1):raise ValueError('SENSOR_TRANSFORM')
  if not self.axes or len(set(self.axes))!=len(self.axes) or any(i not in (0,1,2) for i in self.axes):raise ValueError('SENSOR_AXES')
  self.r=self.T[:3,3]-np.asarray(imu_lever);self.A=self.T[:3,:3].T[list(self.axes)]
 def model(self,x,omega):
  vb=x.R.T@x.v;h=self.A@(vb+np.cross(omega-x.bg,self.r));H=np.zeros((len(self.axes),15));H[:,3:6]=self.A@x.R.T;H[:,6:9]=self.A@skew(vb);H[:,9:12]=self.A@skew(self.r);return h,H
 def correct(self,x,P,omega,velocity,covariance,good=True):
  info=dict(accepted=False,source='RAW_OPTICAL',observed_axes=self.axes,covariance_calibrated=False)
  def fail(reason):return x.copy(),P.copy(),info|dict(reason=reason)
  try:
   valid_prior(P)
   if not good:return fail('OPTICAL_UNQUALIFIED_STATUS')
   z=np.asarray(velocity,float)[list(self.axes)];R=np.asarray(covariance,float)[np.ix_(self.axes,self.axes)];h,H=self.model(x,omega);r=z-h
   if not np.isfinite(np.r_[z,R.ravel(),r]).all() or not np.allclose(R,R.T,atol=1e-12):return fail('OPTICAL_INPUT_INVALID')
   np.linalg.cholesky(R)
   info.update(measurement=z.tolist(),prediction=h.tolist(),innovation=r.tolist(),H=H.tolist(),R=R.tolist())
   # Reuse existing physical velocity-mismatch envelope; no uncalibrated hardNIS.
   if max(abs(r),default=0)>C['wheel_absolute_residual_limit_m_s']:return fail('OPTICAL_SPEED_MISMATCH')
   S=H@P@H.T+R;K=cho_solve(cho_factor(S),H@P).T;d=K@r
   if np.linalg.norm(d[6:9])>C['max_total_rotation_rad'] or np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s'] or np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']:return fail('OPTICAL_UPDATE_DOMAIN')
   A=np.eye(15)-K@H;B=chart_map(d);Q=B@(A@P@A.T+K@R@K.T)@B.T;Q=(Q+Q.T)/2;valid_prior(Q);y=inject(x,d)
  except (ValueError,np.linalg.LinAlgError,IndexError) as e:return fail('OPTICAL_'+str(e))
  return y,Q,info|dict(accepted=True,reason='OPTICAL_VELOCITY_UPDATE',delta=d.tolist(),K=K.tolist(),S=S.tolist(),conditional_NIS=float(r@cho_solve(cho_factor(S),r)))
