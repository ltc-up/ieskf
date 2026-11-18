"""Independent ROS clock process; input decoding/logging cannot hold its GIL."""
import multiprocessing as mp,time,json
from pathlib import Path

def worker(wall,stop,ready,result,path,start,end):
 import rclpy
 from rosgraph_msgs.msg import Clock
 rclpy.init();n=rclpy.create_node('bg_c1_independent_clock');p=n.create_publisher(Clock,'/clock',10);ready.set();prev=None;intervals=[];reversal=False;count=0
 with Path(path).open('x',buffering=1<<20) as f:
  nexttick=time.monotonic_ns()
  while not stop.is_set():
   now=time.monotonic_ns();epoch=wall.value;sim=start if epoch==0 else min(end,start+max(0,now-epoch));m=Clock();m.clock.sec=sim//10**9;m.clock.nanosec=sim%10**9;p.publish(m)
   if epoch:
    if prev is not None:intervals.append(now-prev)
    prev=now;f.write(f'{now},{sim}\n');count+=1
   nexttick+=2_000_000;time.sleep(max(0,(nexttick-time.monotonic_ns())/1e9))
 n.destroy_node();rclpy.shutdown();result.put(dict(ticks=count,max_interval_ms=max(intervals,default=0)/1e6,time_reversal=reversal))
class ClockPump:
 def __init__(self,path,start,end):
  c=mp.get_context('spawn');self.wall=c.Value('q',0);self.stop=c.Event();self.ready=c.Event();self.result=c.Queue();self.p=c.Process(target=worker,args=(self.wall,self.stop,self.ready,self.result,str(path),start,end));self.p.start()
  if not self.ready.wait(10):raise RuntimeError('CLOCK_START_FAILURE')
 def start(self):self.wall.value=time.monotonic_ns()+200_000_000;return self.wall.value
 def close(self):
  self.stop.set();self.p.join(5)
  if self.p.is_alive():self.p.terminate();raise RuntimeError('CLOCK_STOP_FAILURE')
  return self.result.get(timeout=2)
