"""Own-state brake intervention, no geographic/ref data or duplicated raw update."""
import numpy as np
from geometry_ieskf.common import C
from .optical_replay import OpticalReplay
from .delayed_local import DelayedLocalReplay
class FrontendReplay(OpticalReplay):
 def step(self,checkpoint,e):
  if e['kind']!='optical':return super().step(checkpoint,e)
  x,P,imu=checkpoint;u,Q,_=DelayedLocalReplay.step(self,checkpoint,dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']));ax=float((imu['force']-u.ba+u.R.T@np.array(self.predictor.c['gravity_map_m_s2']))[0]);mode=getattr(self,'frontend_mode','F');z=np.array(e['velocity']);V=np.array(e['variance']);used='FILTERED_XY'
  if mode=='B':
   z=np.array(e['raw_velocity']);V=np.diag([.26,.26]);used='RAW_XY'
   if ax< -1.:z[0]=e['filtered_velocity'][0];V[0,0]+=e['frontend']['conditional_variance'][0];used='BRAKE_FILTERED_X'
  raw_pred,_=self.optical.model(u,imu['omega']);raw_res=np.asarray(e.get('raw_velocity',e['velocity']))-raw_pred
  if e['status_good'] and max(abs(raw_res),default=0)>C['wheel_absolute_residual_limit_m_s']:y,S,info=u,Q,dict(accepted=False,reason='RAW_FRONTEND_DISAGREEMENT',raw_innovation=raw_res.tolist())
  else:y,S,info=self.optical.correct(u,Q,imu['omega'],z,V,e['status_good'])
  info.update(frontend_used=used,own_body_acceleration_x=ax,frontend_covariance_calibrated=False);e['last_update']=info;newimu=imu.copy();newimu['last_optical_good_ns']=e['header_ns'] if info['accepted'] else imu.get('last_optical_good_ns',-10**30)
  if hasattr(self,'optical_log'):self.optical_log.append(dict(message_id=e['message_id'],header_ns=e['header_ns'],update=info))
  return y,S,newimu
