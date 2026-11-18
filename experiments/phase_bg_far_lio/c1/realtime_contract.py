"""No-localization ten-second ROS publication/receipt and independent-clock test."""
import argparse,json,time,hashlib,threading
from pathlib import Path
import numpy as np
from continuous_inputs import START,stream,Prefetch,AsyncLog,NAMES,ns
from clock_pump import ClockPump

def main(a):
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from sensor_msgs.msg import Imu,PointCloud2,PointField
 from rclpy.executors import SingleThreadedExecutor
 p=Path(a.root);out=p/'realtime_contract';out.mkdir(exist_ok=False);end=START+10_000_000_000
 audit=json.load(open(a.audit));T=np.array(json.load(open(a.extrinsics))['T_base_imu']);feed=Prefetch(stream(a.bag,audit,T));log=AsyncLog(out/'events.jsonl');clock=ClockPump(out/'clock.csv',START,end)
 rclpy.init();n=rclpy.create_node('bg_c1_no_localization_contract');q=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE);pub=n.create_publisher(PointCloud2,'/c1/test/fused',q);imu=n.create_publisher(Imu,'/c1/test/imu',q);received=[]
 sub=n.create_subscription(PointCloud2,'/c1/test/fused',lambda m:received.append(ns(m.header.stamp)),q)
 ex=SingleThreadedExecutor();ex.add_node(n);th=threading.Thread(target=ex.spin,daemon=True);th.start();time.sleep(.5);epoch=clock.start();sent=[];late=[];reason='COMPLETED'
 try:
  while True:
   e=feed.get()
   if e['available']>end or e['kind']=='end':break
   due=epoch+e['available']-START
   while time.monotonic_ns()<due:time.sleep(min(.001,max(0,(due-time.monotonic_ns())/1e9)))
   if e['kind']=='cloud':
    f=e['data'];m=PointCloud2();m.header.frame_id='base_link';m.header.stamp.sec=e['header']//10**9;m.header.stamp.nanosec=e['header']%10**9;m.height=1;m.width=len(f);m.point_step=f.dtype.itemsize;m.row_step=m.width*m.point_step;m.is_dense=True;m.fields=[PointField(name=k,offset=f.dtype.fields[k][1],datatype=7 if k!='sensor_id' else 2,count=1) for k in f.dtype.names];m.data=f.tobytes();pub.publish(m);sent.append(e['header'])
   elif e['kind']=='imu' and e['adapted'] is not None:
    z=e['adapted'];m=Imu();m.header.frame_id='base_link';m.header.stamp.sec=e['header']//10**9;m.header.stamp.nanosec=e['header']%10**9;m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,z['omega_base']);m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,z['force_base']);imu.publish(m)
   cost=(time.monotonic_ns()-due)/1e6;late.append(cost);log.put(dict(kind=e['kind'],header=e['header'],available=e['available'],lateness_ms=cost))
   if cost>100:raise RuntimeError('SEVERE_PUBLICATION_DELAY')
  time.sleep(.2)
  if sent!=received:raise RuntimeError('ROS_CLOUD_IDENTITY_OR_COMPLETENESS')
 except Exception as exc:reason=repr(exc)
 finally:
  feed.close();log.close();cs=clock.close();ex.shutdown();th.join(2);n.destroy_node();rclpy.shutdown()
 if cs['max_interval_ms']>50:reason='CLOCK_GAP_EXCEEDS_CONTRACT'
 result=dict(reason=reason,sent_clouds=len(sent),received_clouds=len(received),max_publication_delay_ms=max(late,default=0),p95_delay_ms=float(np.percentile(late,95)) if late else None,clock=cs,prefetch_peak_bytes=feed.peak,localization_runs=0)
 (out/'summary.json').write_text(json.dumps(result,indent=2));print(result)
 if reason!='COMPLETED':raise SystemExit(2)
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','bag','audit','extrinsics']];main(p.parse_args())
