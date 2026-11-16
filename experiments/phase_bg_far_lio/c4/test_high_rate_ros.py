"""Synthetic causal 15D owner + read-only100Hz ROS publisher; no real bag/matcher."""
import sys,time,threading,json,hashlib,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'src'))
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.managed_replay import LegacyComparisonReplay
from high_rate_output import SnapshotStore

def serialize(x):return json.dumps(x,sort_keys=True,separators=(',',':'))
def digest(x,P):return hashlib.sha256(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba,P.ravel()].astype('<f8').tobytes()).hexdigest()
def kernel():
 x=State(np.zeros(3),np.array([5.,0,0]),np.eye(3),np.zeros(3),np.zeros(3),0);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);imu=dict(header_ns=0,omega=np.array([0.,0.,.04]),force=np.array([0.,.2,9.81]));return LegacyComparisonReplay(x,P,imu,np.zeros(3))
def events():
 e=[];observer=ExternalPose(np.zeros(3),('x','y','z','rx','ry','rz'))
 for j in range(1,761):
  h=j*5_000_000;e.append((h,dict(kind='imu',header_ns=h,message_id=j,omega=[0,0,.04],force=[0,.2,9.81])))
 for j in range(1,381):
  h=j*10_000_000;e.append((h,dict(kind='wheel',header_ns=h,message_id=10000+j,speed=5.,variance=.01)))
 for j in range(1,76):
  t=j*.05;h=round(t*1e9);R=np.array([[np.cos(.04*t),-np.sin(.04*t),0],[np.sin(.04*t),np.cos(.04*t),0],[0,0,1]]);T=np.eye(4);T[:3,:3]=R;T[:3,3]=[125*np.sin(.04*t),125*(1-np.cos(.04*t)),0];e.append((h+60_000_000,dict(kind='ndt',header_ns=h,message_id=20000+j,correlation_group='synthetic:'+str(j),observation_id='synthetic:'+str(j),observer=observer,pose=T,R=np.diag([.05]*3+[.003]*3))))
 return sorted(e,key=lambda v:(v[0],{'imu':0,'wheel':1,'ndt':2}[v[1]['kind']]))
def stats(x):
 a=np.asarray(x,float);return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max())) if len(a) else {'n':0}
def main(out):
 out=Path(out);k=kernel();expected=[];st=time.perf_counter_ns()
 for available,e in events():k.ingest(e);x,P,_=k.current();expected.append(digest(x,P))
 baseline_ms=(time.perf_counter_ns()-st)/1e6;final0=digest(*k.current()[:2]);store=SnapshotStore();live=kernel();x,P,imu=live.current();store.commit(x,P,0);seen=[];update_ms=[];copy_ms=[];pub=[];recv=[];errors=[];done=threading.Event()
 import rclpy
 from std_msgs.msg import String
 rclpy.init();node=rclpy.create_node('c4_output_contract_synthetic');publisher=node.create_publisher(String,'/c4/research/estimate100',100);subscriber=node.create_subscription(String,'/c4/research/estimate100',lambda m:recv.append((time.monotonic_ns(),json.loads(m.data))),100)
 epoch=time.monotonic_ns()+500_000_000
 def wait(t):
  while True:
   d=t-time.monotonic_ns()
   if d<=0:return
   time.sleep(min(d/1e9,.001))
 def owner():
  lastndt=None
  try:
   for available,e in events():
    wait(epoch+available);tick=time.perf_counter_ns();live.ingest(e);x,P,imu=live.current();update_ms.append((time.perf_counter_ns()-tick)/1e6);seen.append(digest(x,P));lastndt=e['header_ns'] if e['kind']=='ndt' else lastndt;store.commit(x,P,imu['header_ns'],lastndt)
  except Exception as ex:errors.append(repr(ex))
 def output():
  try:
   for seq in range(401):
    scheduled=seq*10_000_000;wait(epoch+scheduled);now=time.monotonic_ns()-epoch
    if now-scheduled>=10_000_000:pub.append(dict(seq=seq,skipped=True,lateness_ms=(now-scheduled)/1e6));continue
    tick=time.perf_counter_ns();packet=store.packet(now,scheduled,seq);cost=(time.perf_counter_ns()-tick)/1e6;msg=String();msg.data=serialize(packet);publisher.publish(msg);pub.append(dict(seq=seq,scheduled_ns=scheduled,actual_ns=now,state_time_ns=packet.get('state_time_ns'),age_ms=packet.get('state_age_ns',0)/1e6,health=packet['health'],revision=packet.get('revision'),skipped=False));copy_ms.append(cost)
  except Exception as ex:errors.append(repr(ex))
  done.set()
 a=threading.Thread(target=owner);b=threading.Thread(target=output);a.start();b.start()
 while not done.is_set() or time.monotonic_ns()<epoch+4_200_000_000:rclpy.spin_once(node,timeout_sec=.002)
 a.join();b.join();node.destroy_node();rclpy.shutdown();final1=digest(*live.current()[:2]);valid=[r for r in pub if not r['skipped']];received=[r['seq'] for _,r in recv];result=dict(scope='real ROS2 synthetic publication; no real dataset accuracy or NDT cycle claim',events=len(seen),event_hash_equal=seen==expected,final_state_P_hash_equal=final0==final1,baseline_offline_wall_ms=baseline_ms,expected_slots=401,published=len(valid),skipped=sum(r['skipped'] for r in pub),received=len(recv),unique_received=len(set(received)),missing_received=sorted(set(r['seq'] for r in valid)-set(received)),stale=sum(r['health']=='STALE' for r in valid),state_age_ms=stats([r['age_ms'] for r in valid]),output_lateness_ms=stats([(r['actual_ns']-r['scheduled_ns'])/1e6 for r in valid]),packet_copy_ms=stats(copy_ms),core_event_update_ms=stats(update_ms),time_reversal=any(v['state_time_ns']<u['state_time_ns'] for u,v in zip(valid,valid[1:])),errors=errors)
 (out/'ros_output_records.jsonl').write_text('\n'.join(json.dumps(r) for r in pub)+'\n');(out/'ros_received_records.jsonl').write_text('\n'.join(json.dumps({'received_wall':t,'packet':r}) for t,r in recv)+'\n');(out/'ros_results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));assert seen==expected and final0==final1 and not errors
if __name__=='__main__':main(sys.argv[1])
