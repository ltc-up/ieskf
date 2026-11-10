"""Measured optical planar speed without treating its direction as certified.
Geometric metric only; first-order noise projection is not statistical calibration.
"""
import numpy as np
from .optical_velocity import OpticalVelocity
from .common import skew
class OpticalPlanarSpeed(OpticalVelocity):
 def raw_speed(self,z):return float(np.linalg.norm(np.asarray(z,float)[:2]))
 def model(self,x,omega):
  A=self.T[:3,:3].T[:2];vb=x.R.T@x.v;h=A@(vb+np.cross(omega-x.bg,self.r));H=np.zeros((2,15));H[:,3:6]=A@x.R.T;H[:,6:9]=A@skew(vb);H[:,9:12]=A@skew(self.r);speed=np.linalg.norm(h)
  if speed<1e-8:raise ValueError('PLANAR_SPEED_UNOBSERVABLE_AT_ZERO')
  return np.array([speed]),(h/speed)[None,:]@H
 def correct(self,x,P,omega,velocity,covariance,good=True):
  z=np.asarray(velocity,float)[:2];speed=np.linalg.norm(z)
  if not good or speed<1e-8:return x.copy(),P.copy(),dict(accepted=False,reason='OPTICAL_BAD_STATUS_OR_ZERO_SPEED')
  n=z/speed;var=float(n@np.asarray(covariance,float)[:2,:2]@n)
  y,Q,info=super().correct(x,P,omega,[speed,0],np.eye(2)*var,True)
  return y,Q,info|dict(measurement_kind='OPTICAL_PLANAR_SPEED',direction_observed=False)
