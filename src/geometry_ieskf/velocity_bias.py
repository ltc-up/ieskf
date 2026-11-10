"""Joint physical15D plus constant optical2D shared error; no GNSS fitting."""
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from .state import State,inject,chart_map
from .inertial import InertialPredictor
from .wheel import WheelObservation
from .external_pose import ExternalPose
from .optical_velocity import OpticalVelocity
from .common import C
class VelocityBiasState(State):
 def __init__(self,p,v,R,bg,ba,t=0,bo=None):super().__init__(p,v,R,bg,ba,t);self.bo=np.zeros(2) if bo is None else np.asarray(bo).copy()
 def copy(self):return VelocityBiasState(*(q.copy() for q in [self.p,self.v,self.R,self.bg,self.ba]),self.t,self.bo.copy())
def lift(x):return VelocityBiasState(x.p.copy(),x.v.copy(),x.R.copy(),x.bg.copy(),x.ba.copy(),x.t,getattr(x,'bo',np.zeros(2)))
def update(x,P,H,r,R,domain=True):
 if not all(np.isfinite(v).all() for v in [P,H,r,R]) or not np.allclose(P,P.T) or not np.allclose(R,R.T):raise ValueError('MATRIX_INVALID')
 np.linalg.cholesky(P);np.linalg.cholesky(R);S=H@P@H.T+R;K=cho_solve(cho_factor(S),H@P).T;d=K@r
 if domain and (np.linalg.norm(d[6:9])>C['max_total_rotation_rad'] or np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s'] or np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']):raise ValueError('UPDATE_DOMAIN')
 A=np.eye(17)-K@H;B=np.eye(17);B[:15,:15]=chart_map(d[:15]);Q=B@(A@P@A.T+K@R@K.T)@B.T;Q=(Q+Q.T)/2;np.linalg.cholesky(Q);y=lift(inject(x,d[:15]));y.bo=x.bo+d[15:];return y,Q,dict(delta=d.tolist(),delta_proposed=d.tolist(),innovation=r.tolist(),K=K.tolist(),R=R.tolist(),S=S.tolist(),optical_bias=y.bo.tolist(),conditional_NIS=float(r@cho_solve(cho_factor(S),r)))
class BiasPredictor(InertialPredictor):
 def predict(self,x,P,omega,force,dt):
  y,F,G,Q=super().model(x,omega,force,dt);A=np.eye(17);A[:15,:15]=F;N=np.zeros((17,17));N[:15,:15]=G@Q@G.T;V=A@P@A.T+N;z=lift(y);z.bo=x.bo.copy();return z,(V+V.T)/2
class BiasOptical(OpticalVelocity):
 def model(self,x,omega):
  h,H=super().model(x,omega);return h+x.bo[list(self.axes)],np.c_[H,np.eye(2)[list(self.axes)]]
 def correct(self,x,P,omega,velocity,covariance,good=True):
  info=dict(accepted=False,covariance_calibrated=False)
  try:
   if not good:return x.copy(),P.copy(),info|dict(reason='OPTICAL_BAD_STATUS')
   h,H=self.model(x,omega);r=np.asarray(velocity)[list(self.axes)]-h
   if max(abs(r),default=0)>C['wheel_absolute_residual_limit_m_s']:return x.copy(),P.copy(),info|dict(reason='OPTICAL_SPEED_MISMATCH')
   y,Q,u=update(x,P,H,r,np.asarray(covariance));return y,Q,info|u|dict(accepted=True,reason='OPTICAL_SHARED_ERROR_UPDATE')
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
class BiasWheel(WheelObservation):
 def model(self,x,omega):
  h,H=super().model(x,omega);return h,np.c_[H,np.zeros((1,2))]
 def correct(self,x,P,omega,z,variance,age=0.):
  h,H=self.model(x,omega);r=np.array([z-h]);info=dict(accepted=False,innovation=float(r[0]))
  try:
   if not np.isfinite([z,variance]).all() or variance<0:return x.copy(),P.copy(),info|dict(reason='INVALID')
   if abs(z)<C['wheel_zero_m_s'] and abs(h)>C['wheel_zero_conflict_prediction_m_s']:return x.copy(),P.copy(),info|dict(reason='ZERO_CONFLICT')
   if abs(r[0])>C['wheel_absolute_residual_limit_m_s']:return x.copy(),P.copy(),info|dict(reason='SLIP_OR_MISMATCH')
   if abs(age)>C['wheel_max_age_s']:return x.copy(),P.copy(),info|dict(reason='STALE')
   # Scaling is a declared unknown-cross-source covariance bound, not fitted noise.
   R=np.array([[(variance+C['wheel_slip_sigma_m_s']**2)*getattr(self,'noise_bound',1.)]]);y,Q,u=update(x,P,H,r,R,False);return y,Q,info|u|dict(accepted=True,reason='ACCEPT')
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
class BiasPose(ExternalPose):
 def correct(self,x,P,T,R,projection=None):
  info=dict(accepted=False,covariance_calibrated=False,components=self.components)
  try:
   r,H=super().residual(x,T);H=np.c_[H,np.zeros((len(H),2))];R=np.asarray(R)
   if projection is not None:r=projection@r;H=projection@H;R=projection@R@projection.T
   y,Q,u=update(x,P,H,r,R);return y,Q,info|u|dict(accepted=True,reason='EXTERNAL_POSE_UPDATE',covariance_updates=1)
  except (ValueError,np.linalg.LinAlgError) as e:return x.copy(),P.copy(),info|dict(reason=str(e))
