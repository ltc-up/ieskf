"""Opt-in real sensor update inside the original bounded15D checkpoint replay."""
from .ar_replay import AuditedPoseReplay
from .delayed_local import DelayedLocalReplay
class OpticalReplay(AuditedPoseReplay):
 @staticmethod
 def key(e):return e['header_ns'],{'imu':0,'wheel':1,'optical':2,'clock':3,'ndt':4}[e['kind']],e['message_id']
 def step(self,checkpoint,e):
  x,P,imu=checkpoint;last=imu.get('last_optical_good_ns',-10**30)
  if e['kind']=='optical':
   advance=dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']);x,P,_=DelayedLocalReplay.step(self,checkpoint,advance);y,Q,info=self.optical.correct(x,P,imu['omega'],e['velocity'],e['variance'],e['status_good']);e['last_update']=info
   newimu=imu.copy();newimu['last_optical_good_ns']=e['header_ns'] if info['accepted'] else last
   if hasattr(self,'optical_log'):self.optical_log.append(dict(message_id=e['message_id'],header_ns=e['header_ns'],update=info))
   return y,Q,newimu
  if e['kind']=='wheel' and self.policy=='optical_xy' and 0<=e['header_ns']-last<=100_000_000:
   advance=dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']);y,Q,_=DelayedLocalReplay.step(self,checkpoint,advance);e['last_update']=dict(accepted=False,reason='ALTERNATIVE_OPTICAL_SOURCE_ACTIVE');return y,Q,imu
  y,Q,newimu=super().step(checkpoint,e);newimu=newimu.copy();newimu['last_optical_good_ns']=last;return y,Q,newimu
