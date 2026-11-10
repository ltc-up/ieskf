"""Joint shared optical error and wheel velocity under real-time brake context."""
import numpy as np
from .ar_replay import AuditedPoseReplay
from .optical_replay import OpticalReplay
from .delayed_local import DelayedLocalReplay
from geometry_ieskf.velocity_bias import lift,BiasPredictor,BiasWheel
class JointVelocityReplay(OpticalReplay):
 def __init__(self,x,P,imu,lever):
  Q=np.zeros((17,17));Q[:15,:15]=P;Q[15:,15:]=np.eye(2)*.25;super().__init__(lift(x),Q,imu,lever);self.predictor=BiasPredictor();self.wheel=BiasWheel(lever)
 def step(self,checkpoint,e):
  x,P,imu=checkpoint;brake=bool(imu.get('braking',False));lastgood=imu.get('last_optical_good_ns',-10**30)
  if e['kind'] in ['wheel','optical']:
   u,Q,_=DelayedLocalReplay.step(self,checkpoint,dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']));ax=float((imu['force']-u.ba+u.R.T@np.asarray(self.predictor.c['gravity_map_m_s2']))[0]);brake=True if ax< -1 else False if ax>-.5 else brake
   if e['kind']=='optical':
    scale=1. if brake else 2.;y,V,info=self.optical.correct(u,Q,imu['omega'],e['velocity'],np.asarray(e['variance'])*scale,e['status_good']);info.update(braking=brake,own_body_acceleration_x=ax,cross_source_noise_bound=scale);e['last_update']=info;newimu=imu.copy();newimu['last_optical_good_ns']=e['header_ns'] if info['accepted'] else lastgood
    if hasattr(self,'optical_log'):self.optical_log.append(dict(message_id=e['message_id'],header_ns=e['header_ns'],update=info))
   elif brake and 0<=e['header_ns']-lastgood<=100_000_000:
    y,V=u,Q;newimu=imu.copy();e['last_update']=dict(accepted=False,reason='BRAKING_WHEEL_RISK',braking=True)
   else:
    self.wheel.noise_bound=2. if 0<=e['header_ns']-lastgood<=100_000_000 else 1.;y,V,info=self.wheel.correct(u,Q,imu['omega'],e['speed'],e['variance']);info.update(braking=brake,cross_source_noise_bound=self.wheel.noise_bound);e['last_update']=info;newimu=imu.copy()
  else:y,V,newimu=AuditedPoseReplay.step(self,checkpoint,e);newimu=newimu.copy();newimu['last_optical_good_ns']=lastgood
  newimu['braking']=brake;return y,V,newimu
