"""Read-only raw bag extraction; payloads must remain in a private output directory."""
import argparse, sqlite3, json, hashlib, collections
from pathlib import Path
import numpy as np

def main(a):
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 from scipy.spatial.transform import Rotation
 con=sqlite3.connect('file:'+str(Path(a.bag).resolve())+'?mode=ro&immutable=1',uri=True)
 topics={n:(i,t) for i,n,t in con.execute('select id,name,type from topics')}
 wanted={'/sensor/bosch/imu':'imu','/localization/wheel_velocity':'wheel','/localization/pose_twist_fusion_filter/kinematic_state':'pose','/localization/ndt_pose':'ndt','/vectornav/raw/gps':'gps','/vectornav/raw/gps2':'gps2','/tf_static':'tf'}
 result=[];summary={};transforms=[]
 for topic,kind in wanted.items():
  if topic not in topics:continue
  tid,typ=topics[topic];cls=get_message(typ);stamps=[];lat=[];dig=hashlib.sha256();fix=collections.Counter()
  for mid,recv,blob in con.execute('select id,timestamp,data from messages where topic_id=? order by timestamp,id',(tid,)):
   dig.update(hashlib.sha256(blob).digest());m=deserialize_message(blob,cls)
   if kind=='tf':
    for tf in m.transforms:
     q=tf.transform.rotation;p=tf.transform.translation
     transforms.append(dict(parent=tf.header.frame_id,child=tf.child_frame_id,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w]))
    continue
   t=m.header.stamp.sec*10**9+m.header.stamp.nanosec;stamps.append(t);lat.append((recv-t)/1e6)
   e=dict(kind=kind,header_ns=t,recorder_ns=recv,message_id=mid,topic=topic,valid=True)
   if kind=='imu':e.update(omega=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],force=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],frame=m.header.frame_id)
   elif kind=='wheel':e.update(speed=m.twist.twist.linear.x,variance=m.twist.covariance[0])
   elif kind in ('pose','ndt'):
    p=m.pose.pose.position;q=m.pose.pose.orientation;e.update(position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],frame=m.header.frame_id)
   else:fix[int(m.fix)]+=1;e.update(fix=int(m.fix),posu=[m.posu.x,m.posu.y,m.posu.z],lla=[m.poslla.x,m.poslla.y,m.poslla.z])
   result.append(e)
  summary[kind]=dict(count=len(stamps),cdr_group_sha256=dig.hexdigest(),max_gap_ms=float(max(np.diff(sorted(set(stamps))),default=0)/1e6),header_rollback=sum(b<a for a,b in zip(stamps,stamps[1:])),receipt_minus_header_ms=dict(zip(['min','p50','p95','p99','max'],np.percentile(lat,[0,50,95,99,100]).tolist())) if lat else {},fix_histogram=dict(fix))
 Path(a.output).write_text(json.dumps(dict(events=result,transforms=transforms,summary=summary)))
 print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);main(p.parse_args())
