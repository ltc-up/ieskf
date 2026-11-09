"""AR instrumentation: unchanged full-P motion equations, explicit health semantics."""
import numpy as np
from .pose_delay import PoseDelayReplay
class AuditedPoseReplay(PoseDelayReplay):
 def step(self,checkpoint,e):
  if e['kind']!='wheel':return super().step(checkpoint,e)
  x,P,imu=checkpoint
  advance=dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force'])
  x,P,_=super(PoseDelayReplay,self).step(checkpoint,advance)
  _,H=self.wheel.model(x,imu['omega']);var=e['variance']+self.wheel.c['wheel_slip_sigma_m_s']**2
  S=float((H@P@H.T)[0,0]+var);K=P@H.T/S
  y,Q,info=self.wheel.correct(x,P,imu['omega'],e['speed'],e['variance'])
  info.update(Kz=float(K[2,0]),Pzz=float(P[2,2]),H=H.tolist(),gain=K[:,0].tolist(),S=S)
  e['last_update']=info;return y,Q,imu

def observation_health(stamp,last_accept,origin,warning_ns=500_000_000,lost_ns=2_000_000_000):
 """Engineering freshness only; TRACKING is not independent peak correctness."""
 if not 0<warning_ns<lost_ns:raise ValueError('HEALTH_INTERVALS')
 age=stamp-(origin if last_accept is None else last_accept)
 if age<0:raise ValueError('TIME_REVERSAL')
 if age>lost_ns:return 'LOST'
 if age>warning_ns:return 'COASTING'
 return 'INITIALIZING' if last_accept is None else 'TRACKING'
