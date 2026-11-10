"""Mechanism-only intervention: omit original wheel information during decel.
No replacement optical or LiDAR-derived velocity enters this run.
"""
import numpy as np
from .ar_replay import AuditedPoseReplay
from .delayed_local import DelayedLocalReplay
class BrakeWheelCounterfactual(AuditedPoseReplay):
 def step(self,checkpoint,e):
  if e['kind']!='wheel':return super().step(checkpoint,e)
  x,P,imu=checkpoint;u,Q,_=DelayedLocalReplay.step(self,checkpoint,dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']));ax=float((imu['force']-u.ba+u.R.T@np.asarray(self.predictor.c['gravity_map_m_s2']))[0])
  if ax< -1.:e['last_update']=dict(accepted=False,reason='BRAKE_WHEEL_COUNTERFACTUAL',own_body_acceleration_x=ax);return u,Q,imu
  return super().step(checkpoint,e)
