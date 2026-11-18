"""Inclusive timing wrappers; no algorithm/clock substitutions."""
import time
from collections import defaultdict
class Timings:
 def __init__(self):self.values=defaultdict(list)
 def call(self,key,fn,*args,**kwargs):
  start=time.perf_counter_ns()
  try:return fn(*args,**kwargs)
  finally:self.values[key].append((time.perf_counter_ns()-start)*1e-6)
 def install(self,namespace):
  replay=namespace['PoseDelayReplay'];observer=namespace['ExternalPose'];timer=self
  class Proxy:
   def __init__(self,obj,name):self.obj=obj;self.name=name
   def __getattr__(self,k):
    value=getattr(self.obj,k)
    if k in ('predict','correct'):return lambda *a,**kw:timer.call(self.name+'_'+k,value,*a,**kw)
    return value
  class Replay(replay):
   def __init__(self,*args,**kwargs):
    super().__init__(*args,**kwargs);self.predictor=Proxy(self.predictor,'imu');self.wheel=Proxy(self.wheel,'wheel')
   def ingest(self,event):return timer.call('ingest_'+event['kind'],super().ingest,event)
  class Pose(observer):
   def correct(self,*args,**kwargs):return timer.call('external_pose_correct',super().correct,*args,**kwargs)
  namespace['PoseDelayReplay']=Replay;namespace['ExternalPose']=Pose
