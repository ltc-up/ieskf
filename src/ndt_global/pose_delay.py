"""External pose events inside AO bounded checkpoint/repropagation, not point factors."""
from .delayed_local import DelayedLocalReplay
class PoseDelayReplay(DelayedLocalReplay):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);self.observation_groups=set()
 def ingest(self,e):
  if e['kind']=='ndt':
   group=e.get('correlation_group')
   if not group:raise ValueError('MISSING_CORRELATION_GROUP')
   if self.key(e) in self.seen:return super().ingest(e)
   if group in self.observation_groups:return 'DUPLICATE_INFORMATION_GROUP'
   result=super().ingest(e)
   if result not in ('OUTSIDE_CHECKPOINT_HISTORY','DUPLICATE'):self.observation_groups.add(group)
   return result
  return super().ingest(e)
 @staticmethod
 def key(e):return e['header_ns'],{'imu':0,'wheel':1,'clock':2,'ndt':3}[e['kind']],e['message_id']
 def step(self,checkpoint,e):
  if e['kind']=='wheel':
   x,P,imu=checkpoint;advance=dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force']);x,P,_=super().step(checkpoint,advance);y,Q,info=self.wheel.correct(x,P,imu['omega'],e['speed'],e['variance']);e['last_update']=info;return y,Q,imu
  if e['kind'] not in ('ndt','clock'):return super().step(checkpoint,e)
  x,P,imu=checkpoint
  # Advance using held IMU by a synthetic IMU event, retaining the original IMU age.
  advance=dict(kind='imu',header_ns=e['header_ns'],omega=imu['omega'],force=imu['force'])
  x,P,_=super().step(checkpoint,advance)
  if e['kind']=='clock':return x,P,imu
  y,Q,info=e['observer'].correct(x,P,e['pose'],e['R'],e.get('projection'));e['last_update']=info
  return y,Q,imu
