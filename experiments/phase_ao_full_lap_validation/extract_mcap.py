"""Read only MCAP filter extraction; no recorder timestamp substitution."""
import argparse,json,hashlib,collections
from pathlib import Path
import numpy as np

def main(a):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 wanted={'/sensor/bosch/imu':'imu','/localization/wheel_velocity':'wheel','/sensor/kistler/twist':'kistler','/localization/pose_twist_fusion_filter/kinematic_state':'pose','/localization/ndt_pose':'ndt','/vectornav/raw/gps':'gps','/vectornav/raw/gps2':'gps2','/tf_static':'tf'}
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={x.name:x.type for x in r.get_all_topics_and_types()};classes={n:get_message(types[n]) for n in wanted if n in types};r.set_filter(rosbag2_py.StorageFilter(topics=list(classes)));events=[];tf=[];dig=collections.defaultdict(hashlib.sha256);mid=0
 while r.has_next():
  topic,blob,recv=r.read_next();mid+=1;m=deserialize_message(blob,classes[topic]);kind=wanted[topic];dig[kind].update(hashlib.sha256(blob).digest())
  if kind=='tf':
   for v in m.transforms:
    p=v.transform.translation;q=v.transform.rotation;tf.append(dict(parent=v.header.frame_id,child=v.child_frame_id,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w]))
   continue
  t=m.header.stamp.sec*10**9+m.header.stamp.nanosec;e=dict(kind=kind,header_ns=t,recorder_ns=recv,message_id=mid,topic=topic,valid=True)
  if kind=='imu':e.update(omega=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],force=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],frame=m.header.frame_id)
  elif kind in ('wheel','kistler'):e.update(speed=m.twist.twist.linear.x,variance=m.twist.covariance[0])
  elif kind in ('pose','ndt'):
   p=m.pose.pose.position;q=m.pose.pose.orientation;e.update(position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],frame=m.header.frame_id)
  else:e.update(fix=int(m.fix),posu=[m.posu.x,m.posu.y,m.posu.z],lla=[m.poslla.x,m.poslla.y,m.poslla.z])
  events.append(e)
 summary={}
 for kind in dig:
  ev=[e for e in events if e['kind']==kind];ts=[e['header_ns'] for e in ev];lat=[(e['recorder_ns']-e['header_ns'])/1e6 for e in ev];summary[kind]=dict(count=len(ev),zero_headers=sum(t==0 for t in ts),cdr_group_sha256=dig[kind].hexdigest(),max_gap_ms=float(max(np.diff(sorted(set(ts))),default=0)/1e6),fix_histogram=dict(collections.Counter(e['fix'] for e in ev if 'fix' in e)),receipt_minus_header_ms=dict(zip(['min','p50','p95','p99','max'],np.percentile(lat,[0,50,95,99,100]).tolist())) if lat else {})
 Path(a.output).write_text(json.dumps(dict(events=events,transforms=tf,summary=summary)));print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);main(p.parse_args())
