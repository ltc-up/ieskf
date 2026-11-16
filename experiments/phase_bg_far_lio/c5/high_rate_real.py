"""C4 SnapshotStore reused unchanged, with real-input freshness metadata/ROS IO."""
import sys,threading,time,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'c4'))
from high_rate_output import SnapshotStore
class RealOutput:
 def __init__(self,out,start,end,logclass):
  import rclpy
  from std_msgs.msg import String
  self.rclpy=rclpy;self.String=String;self.start_ns=start;self.end_ns=end;self.store=SnapshotStore();self.lock=threading.Lock();self.aux={};self.records=logclass(out/'output100.jsonl');self.acks=logclass(out/'output100_receipts.jsonl');self.samples=[];self.received=[];self.errors=[];self.stop=False;self.abort=threading.Event()
  rclpy.init();self.node=rclpy.create_node('bg_c5_readonly_output');self.pub=self.node.create_publisher(String,'/c5/research/estimate100',100)
  def receive(m):
   wall=time.monotonic_ns();v=json.loads(m.data);self.received.append(v['seq']);self.acks.put(dict(seq=v['seq'],revision=v.get('revision'),received_wall_ns=wall,loopback_includes_serialization_ms=(wall-v['send_wall_ns'])/1e6))
  self.sub=self.node.create_subscription(String,'/c5/research/estimate100',receive,100);self.spin=threading.Thread(target=self.spinloop,daemon=True);self.spin.start()
 def spinloop(self):
  while not self.stop:self.rclpy.spin_once(self.node,timeout_sec=.002)
 def commit(self,x,P,imu,wheel,lidar,attempt,replayed):
  with self.lock:
   self.store.commit(x,P,imu,lidar);self.aux=dict(last_wheel_ns=wheel,last_ndt_attempt_ns=attempt,last_accepted_ndt_ns=lidar,replayed_events=replayed)
 def start(self,epoch):
  self.epoch=epoch
  def run():
   for seq in range((self.end_ns-self.start_ns)//10_000_000+1):
    if self.abort.is_set():return
    scheduled=self.start_ns+seq*10_000_000;due=epoch+scheduled-self.start_ns
    while time.monotonic_ns()<due:
     if self.abort.is_set():return
     time.sleep(min(.001,max(0,(due-time.monotonic_ns())/1e9)))
    wall=time.monotonic_ns();now=self.start_ns+wall-epoch;t=time.perf_counter_ns()
    with self.lock:packet=self.store.packet(now,scheduled,seq);packet.update(self.aux)
    copyms=(time.perf_counter_ns()-t)/1e6;lid=packet.get('last_accepted_ndt_ns');wh=packet.get('last_wheel_ns');packet.update(wheel_age_ns=None if wh is None else now-wh,map_observation_age_ns=None if lid is None else now-lid,map_health='NO_ACCEPTED_MAP_OBSERVATION' if lid is None else 'MAP_UPDATE_STALE' if now-lid>500_000_000 else 'RECENT_MAP_UPDATE_UNCERTIFIED',map_branch_correctness='UNKNOWN',send_wall_ns=time.monotonic_ns())
    t=time.perf_counter_ns();m=self.String();m.data=json.dumps(packet);ser=(time.perf_counter_ns()-t)/1e6;t=time.perf_counter_ns();self.pub.publish(m);pub=(time.perf_counter_ns()-t)/1e6
    self.records.put(dict(packet=packet,copy_ms=copyms,serialization_ms=ser,publish_API_ms=pub,deadline_start_lateness_ms=(wall-due)/1e6));self.samples.append((seq,packet.get('state_age_ns'),packet['health'],(wall-due)/1e6,copyms,ser,pub,packet.get('state_time_ns')))
  def guarded():
   try:run()
   except Exception as ex:self.errors.append(repr(ex))
  self.thread=threading.Thread(target=guarded,daemon=True);self.thread.start()
 def close(self,abort=False):
  if abort:self.abort.set()
  self.thread.join(10)
  if self.thread.is_alive():raise RuntimeError('OUTPUT100_NOT_FINISHED')
  time.sleep(.2);self.stop=True;self.spin.join(2);self.node.destroy_node();self.rclpy.shutdown();self.records.close();self.acks.close()
  def stat(v):
   a=np.array(v,float);return {} if not len(a) else dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()),min=float(a.min()))
  return dict(errors=self.errors,aborted=abort,expected=(self.end_ns-self.start_ns)//10_000_000+1,published=len(self.samples),received=len(self.received),unique_received=len(set(self.received)),missing=sorted(set(x[0] for x in self.samples)-set(self.received)),stale=sum(x[2]=='STALE' for x in self.samples),uninitialized=sum(x[2]=='UNINITIALIZED' for x in self.samples),time_reversal=any(b[7]<a[7] for a,b in zip([q for q in self.samples if q[7] is not None],[q for q in self.samples if q[7] is not None][1:])),state_age_ms=stat([q[1]/1e6 for q in self.samples if q[1] is not None]),deadline_lateness_ms=stat([q[3] for q in self.samples]),copy_ms=stat([q[4] for q in self.samples]),serialize_ms=stat([q[5] for q in self.samples]),publish_API_ms=stat([q[6] for q in self.samples]),late_slots_over_10ms=sum(q[3]>10 for q in self.samples))
