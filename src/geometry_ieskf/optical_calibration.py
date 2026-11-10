"""Opt-in static optical mounting yaw as a correlated estimated state.
Base15D motion equations unchanged;16th error is a constant sensor yaw.
"""
import numpy as np
from .state import State,inject,chart_map
from .common import skew,exp,C
from .inertial import InertialPredictor
from .wheel import WheelObservation
from .external_pose import ExternalPose
from .optical_velocity import OpticalVelocity
from scipy.linalg import cho_factor,cho_solve
class CalibratedState(State):
 def __init__(self,p,v,R,bg,ba,t=0,beta=0.):super().__init__(p,v,R,bg,ba,t);self.beta=float(beta)
 def copy(self):return CalibratedState(*(a.copy() for a in [self.p,self.v,self.R,self.bg,self.ba]),self.t,self.beta)
def lift(x,beta=None):return CalibratedState(x.p.copy(),x.v.copy(),x.R.copy(),x.bg.copy(),x.ba.copy(),x.t,getattr(x,'beta',0.) if beta is None else beta)
def inject16(x,d):return lift(inject(x,np.asarray(d)[:15]),x.beta+d[15])
def correct16(x,P,H,r,R,domain=True):
 if not all(np.isfinite(v).all() for v in [P,H,r,R]) or not np.allclose(P,P.T,atol=1e-12) or not np.allclose(R,R.T,atol=1e-12):raise ValueError('CALIBRATED_MATRIX_INVALID')
 np.linalg.cholesky(P);np.linalg.cholesky(R)
 S=H@P@H.T+R;K=cho_solve(cho_factor(S),H@P).T;d=K@r
 if domain and (np.linalg.norm(d[6:9])>C['max_total_rotation_rad'] or np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s'] or np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']):raise ValueError('CALIBRATED_UPDATE_DOMAIN')
 if not np.isfinite(d).all():raise ValueError('NONFINITE')
 A=np.eye(16)-K@H;B=np.eye(16);B[:15,:15]=chart_map(d[:15]);Q=B@(A@P@A.T+K@R@K.T)@B.T;Q=(Q+Q.T)/2;np.linalg.cholesky(Q);return inject16(x,d),Q,dict(delta=d.tolist(),delta_proposed=d.tolist(),K=K.tolist(),innovation=r.tolist(),R=R.tolist(),S=S.tolist(),conditional_NIS=float(r@cho_solve(cho_factor(S),r)),calibration_yaw_rad=float(x.beta+d[15]))
class CalibratedPredictor(InertialPredictor):
 def predict(self,x,P,omega,force,dt):
  y,F,G,Q=super().model(x,omega,force,dt);A=np.eye(16);A[:15,:15]=F;N=np.zeros((16,16));N[:15,:15]=G@Q@G.T;out=A@P@A.T+N;return lift(y,x.beta),(out+out.T)/2
class CalibratedWheel(WheelObservation):
 def model(self,x,omega):
  h,H=super().model(x,omega);return h,np.c_[H,np.zeros(1)]
 def correct(self,x,P,omega,z,variance,age=0.):
  h,H=self.model(x,omega);r=float(z-h);info=dict(predicted=float(h),measurement=float(z),innovation=r)
  if not np.isfinite([z,variance]).all() or variance<0:return x.copy(),P.copy(),info|dict(accepted=False,reason='INVALID')
  if abs(age)>C['wheel_max_age_s']:return x.copy(),P.copy(),info|dict(accepted=False,reason='STALE')
  if abs(z)<C['wheel_zero_m_s'] and abs(h)>C['wheel_zero_conflict_prediction_m_s']:return x.copy(),P.copy(),info|dict(accepted=False,reason='ZERO_CONFLICT')
  if abs(r)>C['wheel_absolute_residual_limit_m_s']:return x.copy(),P.copy(),info|dict(accepted=False,reason='SLIP_OR_MISMATCH')
  try:y,Q,u=correct16(x,P,H,np.array([r]),np.array([[variance+C['wheel_slip_sigma_m_s']**2]]),False)
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(accepted=False,reason=str(e))
  return y,Q,info|u|dict(accepted=True,reason='ACCEPT')
class CalibratedOptical(OpticalVelocity):
 def model(self,x,omega):
  Rs=self.T[:3,:3]@exp(np.array([0.,0.,x.beta]));A=Rs.T[list(self.axes)];vb=x.R.T@x.v;u=vb+np.cross(omega-x.bg,self.r);h=A@u;H=np.zeros((len(self.axes),16));H[:,3:6]=A@x.R.T;H[:,6:9]=A@skew(vb);H[:,9:12]=A@skew(self.r);H[:,15]=(-skew(np.array([0.,0.,1.]))@Rs.T@u)[list(self.axes)];return h,H
 def correct(self,x,P,omega,velocity,covariance,good=True):
  info=dict(accepted=False,source='RAW_OPTICAL_CALIBRATED',observed_axes=self.axes,covariance_calibrated=False)
  try:
   if not good:return x.copy(),P.copy(),info|dict(reason='OPTICAL_UNQUALIFIED_STATUS')
   z=np.asarray(velocity)[list(self.axes)];R=np.asarray(covariance)[np.ix_(self.axes,self.axes)];np.linalg.cholesky(R);h,H=self.model(x,omega);r=z-h;info.update(measurement=z.tolist(),prediction=h.tolist(),innovation=r.tolist(),H=H.tolist())
   if max(abs(r),default=0)>C['wheel_absolute_residual_limit_m_s']:return x.copy(),P.copy(),info|dict(reason='OPTICAL_SPEED_MISMATCH')
   y,Q,u=correct16(x,P,H,r,R)
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
  return y,Q,info|u|dict(accepted=True,reason='OPTICAL_VELOCITY_UPDATE')
class CalibratedPose(ExternalPose):
 def correct(self,x,P,T,R,projection=None):
  info=dict(accepted=False,covariance_calibrated=False,components=self.components)
  try:
   r,H=super().residual(x,T);H=np.c_[H,np.zeros(len(H))];R=np.asarray(R)
   if projection is not None:r=projection@r;H=projection@H;R=projection@R@projection.T
   np.linalg.cholesky(R);y,Q,u=correct16(x,P,H,r,R)
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
  return y,Q,info|u|dict(accepted=True,reason='EXTERNAL_POSE_UPDATE',covariance_updates=1)
