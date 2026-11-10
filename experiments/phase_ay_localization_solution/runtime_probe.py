"""Inclusive measurements only; never substitutes time or drops input frames."""
import gc,time,resource,json
from pathlib import Path
from collections import defaultdict
class RuntimeProbe:
 def __init__(self):self.frame=-1;self.events=[];self.frames=[];self.start={};self.modules=defaultdict(list)
 def callback(self,phase,info):
  gen=info['generation']
  if phase=='start':self.start[gen]=time.perf_counter_ns()
  else:self.events.append(dict(frame=self.frame,generation=gen,ms=(time.perf_counter_ns()-self.start.pop(gen,time.perf_counter_ns()))*1e-6,collected=info['collected']))
 def record(self,index,start,cycle_start=None):
  self.frames.append(dict(index=index,cycle_ms=(time.perf_counter_ns()-(start if cycle_start is None else cycle_start))*1e-6,ms=(time.perf_counter_ns()-start)*1e-6,maxrss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
 def call(self,name,fn,*args,**kwargs):
  tick=time.perf_counter_ns()
  try:return fn(*args,**kwargs)
  finally:self.modules[name].append((time.perf_counter_ns()-tick)*1e-6)
 def install(self,ns):
  from profile_runtime import Timings
  self.timers=Timings();self.timers.install(ns);parent=ns['OpticalVelocity'];probe=self
  class Optical(parent):
   def correct(self,*args,**kwargs):return probe.call('optical_correct',super().correct,*args,**kwargs)
  ns['OpticalVelocity']=Optical;gc.callbacks.append(self.callback)
 def save(self,path):
  gc.callbacks.remove(self.callback);Path(path).write_text(json.dumps(dict(frames=self.frames,gc_events=self.events,modules=dict(self.timers.values)|dict(self.modules))))
