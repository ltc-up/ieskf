"""Independent XY/yaw pose observation for the full right-error inertial state.

Historical engineering R, not calibrated noise. No point factors or NDT Hessian.
"""
import time
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from .common import C,skew
from .state import inject,chart_map
from .posterior import valid_prior
from .partition import margins

def wrap(a):return np.arctan2(np.sin(a),np.cos(a))
def pose_xyyaw(T):
 T=np.asarray(T);return np.r_[T[:2,3],np.arctan2(T[1,0],T[0,0])]

def engineering_R(fitness,smoothing_steps=5):
 """Frozen legacy per-update XY/yaw rule; no repeated queue smoothing here.

 Fitness-derived translation variance f and angular variance f/2, legacy
 low/high diagonal scaling, then the historical smoothing multiplier.
 """
 f=float(fitness)
 if not np.isfinite(f) or f<0 or f>15:raise ValueError('INVALID_HISTORICAL_FITNESS_COVARIANCE')
 f=max(f,1e-5)  # Frozen publisher's legacy floor, not a tuned R scale.
 xy,yaw=(1.5*f,.6*f) if f>=3 else (.05*f,.00025*f)
 return np.diag([xy,xy,yaw])*smoothing_steps

class NdtPoseObservation:
 def __init__(self,lever,config=C):self.lever=np.asarray(lever);self.c=config
 def model(self,x):
  u=x.R[:,0];den=u[0]**2+u[1]**2
  if den<=np.finfo(float).eps:raise ValueError('YAW_CHART_SINGULAR')
  H=np.zeros((3,15));H[:2,:3]=np.eye(3)[:2];H[:2,6:9]=(x.R@skew(self.lever))[:2]
  H[2,6:9]=(np.array([-u[1],u[0],0.])/den)@(-x.R@skew(np.array([1.,0.,0.])))
  pbase=x.p-x.R@self.lever
  return np.r_[pbase[:2],np.arctan2(u[1],u[0])],H
 def correct(self,x,P,measurement,R):
  tick=time.perf_counter();info={'accepted':False,'covariance_updates':0,'reason':'UNINITIALIZED'}
  def keep(reason):
   info.update(reason=reason,wall_ms=1000*(time.perf_counter()-tick));return x.copy(),np.array(P,copy=True),info
  try:
   valid_prior(P);R=np.asarray(R);z=np.asarray(measurement)
   if R.shape!=(3,3) or z.shape!=(3,) or not np.isfinite(np.r_[R.ravel(),z]).all():return keep('INVALID_OBSERVATION')
   if np.max(abs(R-R.T))>1e-12:return keep('INVALID_R')
   np.linalg.cholesky(R);h,H=self.model(x);res=z-h;res[2]=wrap(res[2]);S=H@P@H.T+R;K=cho_solve(cho_factor(S),H@P).T;d=K@res
   info.update(innovation=res.tolist(),delta_proposed=d.tolist(),K=K.tolist(),R=R.tolist(),predicted_observation=h.tolist(),nis_model_conditional=float(res@cho_solve(cho_factor(S),res)))
   failures=[]
   if not np.isfinite(d).all():return keep('NONFINITE_DELTA')
   if np.linalg.norm(d[:3])>self.c['max_translation_step_m']:failures.append('POSITION_SINGLE_STEP')
   if np.linalg.norm(d[6:9])>self.c['max_rotation_step_rad']:failures.append('ORIENTATION_SINGLE_STEP')
   for name,m in zip(['POSITION_TOTAL','ORIENTATION_TOTAL','GYRO_BIAS','ACCEL_BIAS'],margins(d)):
    if m<0:failures.append(name)
   info['guard_failures']=failures
   if failures:return keep('ENGINEERING_GUARD')
   A=np.eye(15)-K@H;Pc=A@P@A.T+K@R@K.T;B=chart_map(d);post=B@Pc@B.T;post=(post+post.T)/2;valid_prior(post);candidate=inject(x,d)
   if not np.isfinite(np.r_[candidate.p,candidate.v,candidate.R.ravel(),candidate.bg,candidate.ba]).all():return keep('NONFINITE_STATE')
  except (ValueError,np.linalg.LinAlgError):return keep('NUMERIC_OR_INPUT_FAILURE')
  info.update(accepted=True,reason='ACCEPTED_NDT_XY_YAW',delta=d.tolist(),covariance_updates=1,wall_ms=1000*(time.perf_counter()-tick),NUMERIC_POSTERIOR_VALID=True,STATISTICAL_POSTERIOR_CALIBRATED=False)
  return candidate,post,info
