"""Passive observer diagnostics. Calls the unchanged correct exactly once."""
import time
import numpy as np
from geometry_ieskf.external_pose import ExternalPose
LOG=None
WINDOW=None
class TraceObserver(ExternalPose):
 def __init__(self,lever,uid):super().__init__(lever,('x','y','z','rx','ry','rz'));self.uid=uid
 def correct(self,x,P,T,R,projection=None):
  y,Q,info=super().correct(x,P,T,R,projection)
  if LOG is not None and WINDOW[0]<=x.t<=WINDOW[1]:
   r,H=self.residual(x,T);S=H@P@H.T+R
   LOG.put(dict(uid=self.uid,state_time_ns=x.t,wall_ns=time.monotonic_ns(),prior=dict(p=x.p,v=x.v,R=x.R,bg=x.bg,ba=x.ba,P=P),H=H,S=S,R=R,update=info,posterior=dict(p=y.p,v=y.v,R=y.R,bg=y.bg,ba=y.ba,P=Q)))
  return y,Q,info
