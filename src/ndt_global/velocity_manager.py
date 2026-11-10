"""Causal source/axis selection; no parallel independent forward likelihoods.
No GNSS, geographic index or retrospective measurement enters this manager.
"""
import numpy as np
from .ar_replay import AuditedPoseReplay
from .delayed_local import DelayedLocalReplay
from geometry_ieskf.common import C
class ManagedVelocityReplay(AuditedPoseReplay):
 TIMEOUT_NS=100_000_000
 RECOVERY_NS=100_000_000
 LOW_SPEED=1.0
 @staticmethod
 def key(e):return e['header_ns'],{'imu':0,'wheel':1,'optical':2,'clock':3,'ndt':4}[e['kind']],e['message_id']
 def step(self,checkpoint,e):
  x,P,imu=checkpoint;t=e['header_ns'];meta=dict(imu.get('velocity_manager',{}));phase=meta.get('phase','INITIALIZING');last=meta.get('last_good_ns',-10**30)
  if phase=='OPTICAL_ACTIVE' and t-last>self.TIMEOUT_NS:phase='STALE';meta['phase']=phase;meta.pop('recovery_start',None)
  if e['kind']=='optical':
   advance=dict(kind='imu',header_ns=t,omega=imu['omega'],force=imu['force']);u,Q,_=DelayedLocalReplay.step(self,checkpoint,advance);z=np.asarray(e['velocity'],float).copy();z[0]=self.optical.raw_speed(z) if hasattr(self.optical,'raw_speed') else z[0];pred=self.optical.model(u,imu['omega'])[0][0];reason=None
   if not e['status_good'] or not np.isfinite(z).all():reason='OPTICAL_BAD_STATUS'
   if reason is None and abs(z[0])<1e-3 and abs(meta.get('wheel_speed',0))>5 and 0<=t-meta.get('wheel_ns',-10**30)<=self.TIMEOUT_NS:reason='OPTICAL_ZERO_CONFLICT'
   # Difference of successive innovations cancels constant sensor offset. Existing
   # 5m/s envelope is reused; this is a gross fault rule, not a calibrated NIS.
   if reason is None and 'last_raw' in meta and 0<t-meta['last_raw_ns']<=self.TIMEOUT_NS:
    change=(z[0]-meta['last_raw'])-(pred-meta['last_pred'])
    if abs(change)>C['wheel_absolute_residual_limit_m_s']:reason='OPTICAL_DYNAMIC_JUMP'
   if reason is not None:
    y,V=u,Q;phase='FAULT';meta.pop('recovery_start',None);info=dict(accepted=False,reason=reason)
   elif abs(z[0])<self.LOW_SPEED:
    y,V=u,Q;phase='LOW_SPEED_WHEEL';meta.pop('recovery_start',None);info=dict(accepted=False,reason='LOW_SPEED_WHEEL')
   else:
    # Fresh consecutive good data, not merely a bit changing back to OK.
    if phase!='OPTICAL_ACTIVE':
     if t-meta.get('last_packet_ns',t)>self.TIMEOUT_NS:meta.pop('recovery_start',None)
     meta.setdefault('recovery_start',t)
    ready=phase=='OPTICAL_ACTIVE' or t-meta['recovery_start']>=self.RECOVERY_NS
    if ready:
     y,V,info=self.optical.correct(u,Q,imu['omega'],e['velocity'],e['variance'],True)
     if info['accepted']:phase='OPTICAL_ACTIVE';meta['last_good_ns']=t
     else:phase='FAULT';meta.pop('recovery_start',None)
    else:y,V=u,Q;phase='RECOVERING';info=dict(accepted=False,reason='OPTICAL_RECOVERING')
    if phase!='FAULT':meta.update(last_raw=float(z[0]),last_pred=float(pred),last_raw_ns=t)
   meta.update(phase=phase,last_packet_ns=t);newimu=imu.copy();newimu['velocity_manager']=meta;e['last_update']=info|dict(source_phase=phase,observed_axes=[0])
   if hasattr(self,'optical_log'):self.optical_log.append(dict(message_id=e['message_id'],header_ns=t,update=e['last_update']))
  elif e['kind']=='wheel':
   meta.update(wheel_speed=float(e['speed']),wheel_ns=t)
   if phase=='OPTICAL_ACTIVE' and 0<=t-last<=self.TIMEOUT_NS:
    y,V,_=DelayedLocalReplay.step(self,checkpoint,dict(kind='imu',header_ns=t,omega=imu['omega'],force=imu['force']));newimu=imu.copy();e['last_update']=dict(accepted=False,reason='OPTICAL_FORWARD_EXCLUSIVE',source_phase=phase)
   else:y,V,newimu=super().step(checkpoint,e)
   newimu=newimu.copy();newimu['velocity_manager']=meta
  else:
   y,V,newimu=super().step(checkpoint,e);newimu=newimu.copy();newimu['velocity_manager']=meta
  if e['kind'] in ['wheel','optical'] and hasattr(self,'manager_log'):
   self.manager_log.append(dict(message_id=e['message_id'],header_ns=t,kind=e['kind'],phase=meta.get('phase',phase),update=e.get('last_update',{}),P_before=np.diag(P).tolist(),P_after=np.diag(V).tolist(),state_before=dict(v=x.v.tolist(),R=x.R.tolist(),bg=x.bg.tolist(),ba=x.ba.tolist()),state_after=dict(v=y.v.tolist(),R=y.R.tolist(),bg=y.bg.tolist(),ba=y.ba.tolist())))
  return y,V,newimu
