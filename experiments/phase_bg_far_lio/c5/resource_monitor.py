"""One-second sampled process tree/GPU/disk audit; explicit resource failure."""
import os,time,threading,subprocess
from pathlib import Path
import psutil
class ResourceMonitor:
 def __init__(self,root,limits,logger):
  self.root=Path(root);self.limits=limits;self.log=logger;self.error=None;self.stop=False;self.peak_rss=0;self.peak_vram=0;self.peak_bytes=0;self.cpu_previous={}
  self.proc=psutil.Process(os.getpid());self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
 def run(self):
  while not self.stop:
   try:
    procs=[self.proc]+self.proc.children(recursive=True);rss=sum(p.memory_info().rss for p in procs if p.is_running());cpu=0.;now=time.monotonic()
    for proc in procs:
     if not proc.is_running():continue
     ct=proc.cpu_times();total=ct.user+ct.system;old=self.cpu_previous.get(proc.pid)
     if old is not None and now>old[1]:cpu+=100*(total-old[0])/(now-old[1])
     self.cpu_previous[proc.pid]=(total,now)
    b=sum(p.stat().st_size for p in self.root.rglob('*') if p.is_file());cmd=['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits'];r=subprocess.run(cmd,capture_output=True,text=True,timeout=2);mem,util=[float(x.strip()) for x in r.stdout.splitlines()[0].split(',')] if r.returncode==0 else (0,0)
    self.peak_rss=max(self.peak_rss,rss);self.peak_vram=max(self.peak_vram,mem);self.peak_bytes=max(self.peak_bytes,b);self.log.put(dict(wall_ns=time.monotonic_ns(),tree_rss_bytes=rss,process_tree_cpu_percent_sample=cpu,device_vram_MiB=mem,device_gpu_percent=util,files_apparent_bytes=b))
    if rss>self.limits['RSS_cap_GiB']*1024**3 or mem>self.limits['VRAM_cap_GiB']*1024 or b>self.limits['disk_cap_bytes']:self.error='RESOURCE_LIMIT_EXCEEDED';return
   except psutil.NoSuchProcess:pass
   except Exception as e:self.error='RESOURCE_MONITOR:'+repr(e);return
   time.sleep(1)
 def close(self):
  self.stop=True;self.thread.join(3);self.log.close();return dict(sample_interval_s=1,peak_process_tree_RSS_bytes=self.peak_rss,peak_device_VRAM_MiB=self.peak_vram,peak_apparent_bytes=self.peak_bytes,error=self.error,not_continuous_hardware_peak=True)
