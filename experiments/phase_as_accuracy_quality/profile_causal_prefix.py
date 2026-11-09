"""Instrument a bounded real causal prefix; AR estimator/config bytes unchanged.
Inclusive nested timing categories overlap; never sum their percentiles.
"""
import argparse,collections,functools,gc,importlib.util,json,resource,time
from pathlib import Path
class Timing:
 def __init__(self):self.frame=-1;self.active=False;self.start=None;self.records=[];self.frames=[];self.gc_start=None;self.gc_events=[]
 def record_frame(self):
  if self.start is not None:self.frames.append(dict(frame=self.frame,wall_ms=(time.perf_counter_ns()-self.start)/1e6))
 def new_frame(self,i):self.record_frame();self.frame=i;self.active=True;self.start=time.perf_counter_ns()
 def end_frames(self):self.record_frame();self.start=None;self.active=False
 def call(self,name,fn,*args,**kwargs):
  t=time.perf_counter_ns()
  try:return fn(*args,**kwargs)
  finally:self.records.append(dict(frame=self.frame,phase='frame' if self.active else 'startup_or_finalize',name=name,ms=(time.perf_counter_ns()-t)/1e6))
 def measure(self,name):
  def deco(fn):
   @functools.wraps(fn)
   def wrap(*args,**kwargs):return self.call(name,fn,*args,**kwargs)
   return wrap
  return deco
 def gc(self,phase,info):
  if phase=='start':self.gc_start=(time.perf_counter_ns(),self.frame,info['generation'])
  elif self.gc_start:
   t,f,g=self.gc_start;self.gc_events.append(dict(frame=f,generation=g,ms=(time.perf_counter_ns()-t)/1e6));self.gc_start=None
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--limit',type=int,default=850);p.add_argument('--window',default='lap');a=p.parse_args();path=Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py';s=path.read_text();s=s.replace(' def advance(until):'," @timing.measure('advance')\n def advance(until):").replace(' def checkpoint(stamp):'," @timing.measure('checkpoint')\n def checkpoint(stamp):").replace('replay.ingest(e)',"timing.call('sensor_ingest',replay.ingest,e)").replace('reason=replay.ingest(event)',"reason=timing.call('ndt_ingest',replay.ingest,event)").replace('ack=deskproc.stdout.readline()',"ack=timing.call('deskew_ipc',deskproc.stdout.readline)").replace('line=proc.stdout.readline()',"line=timing.call('matcher_ipc',proc.stdout.readline)").replace("sha(Path(c['raw_path']))","timing.call('raw_file_hash',sha,Path(c['raw_path']))").replace(' for i,job in enumerate(jobs):',' for i,job in enumerate(jobs):\n  timing.new_frame(i)').replace(" (a.private/'transactions.json').write_text", " timing.end_frames()\n (a.private/'transactions.json').write_text")
 timing=Timing();ns={'__name__':'instrumented_ar','timing':timing};exec(compile(s,str(path),'exec'),ns);a.mode='F';a.stop_loss=None;old_dumps=json.dumps;old_write=Path.write_text;gc.callbacks.append(timing.gc);json.dumps=lambda *args,**kwargs:timing.call('json_dumps',old_dumps,*args,**kwargs);Path.write_text=lambda *args,**kwargs:timing.call('write_text',old_write,*args,**kwargs)
 try:ns['main'](a)
 finally:
  json.dumps=old_dumps;Path.write_text=old_write;gc.callbacks.remove(timing.gc);a.private.mkdir(parents=True,exist_ok=True);(a.private/'timing_records.json').write_text(json.dumps(dict(records=timing.records,frames=timing.frames,gc=timing.gc_events,resource_maxrss_kb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),indent=2))
