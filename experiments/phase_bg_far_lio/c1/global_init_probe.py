"""One offline FULL_MAP GlobalInit attempt. No navigation/reference input topics.
Original package algorithm is reused; selection uses time, synchronization and
stationarity only. Private payloads are never written by this runner.
"""
import argparse,hashlib,json,os,signal,sqlite3,subprocess,time
from pathlib import Path
import numpy as np

def stamp(m,n):m.sec=int(n)//10**9;m.nanosec=int(n)%10**9
def ns(t):return int(t.sec)*10**9+int(t.nanosec)
def main(a):
 import rclpy
 from rclpy.serialization import deserialize_message
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from sensor_msgs.msg import PointCloud2
 from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped,Pose
 from std_msgs.msg import Bool
 from rosgraph_msgs.msg import Clock
 from tf2_ros import StaticTransformBroadcaster
 p=Path(a.root);out=p/'initialization';out.mkdir(exist_ok=False)
 db=sqlite3.connect('file:'+a.bag+'?mode=ro&immutable=1',uri=True)
 origin=db.execute('select min(timestamp) from messages').fetchone()[0]
 topics={n:i for i,n in db.execute('select id,name from topics')}
 names=['/sensor/lidar_front/points','/sensor/lidar_left/points','/sensor/lidar_right/points']
 slots={n:[] for n in names}
 lo=origin+135_000_000_000
 for n in names:
  for ident,available,payload in db.execute('select id,timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<? order by timestamp',(topics[n],lo,lo+200_000_000)):
   m=deserialize_message(payload,PointCloud2);slots[n].append((ident,available,m))
 front=slots[names[0]][0];h=ns(front[2].header.stamp)
 chosen=[front]+[min(slots[n],key=lambda v:abs(ns(v[2].header.stamp)-h)) for n in names[1:]]
 headers=[ns(v[2].header.stamp) for v in chosen]
 if max(headers)-min(headers)>10_000_000:raise RuntimeError('SYNC_CONTRACT')
 # IMU/wheel only. Never consume recorded pose, GNSS or engineering reference.
 motion=json.load(open(a.motion));events=motion['events'];imu=[e for e in events if e['kind']=='imu' and lo-5_000_000_000<=e['header_ns']<=lo+1_000_000_000];wheel=[e for e in events if e['kind']=='wheel' and lo-5_000_000_000<=e['header_ns']<=lo+1_000_000_000]
 w=np.array([e['speed'] for e in wheel]);g=np.array([e['omega'] for e in imu]);f=np.array([e['force'] for e in imu]);dt=np.diff([e['header_ns'] for e in imu])
 stats=dict(imu_count=len(imu),wheel_count=len(wheel),wheel_abs_max=float(abs(w).max()),gyro_norm_max=float(np.linalg.norm(g,axis=1).max()),gyro_mean=g.mean(0).tolist(),gyro_std=g.std(0).tolist(),force_mean=f.mean(0).tolist(),force_std=f.std(0).tolist(),force_norm_median=float(np.median(np.linalg.norm(f,axis=1))),imu_dt_max_ms=float(dt.max()/1e6))
 if len(imu)<1000 or len(wheel)<500 or abs(w).max()>.01 or np.linalg.norm(g,axis=1).max()>.03 or f.std(0).max()>.5 or dt.max()>30_000_000:raise RuntimeError('STATIONARITY_CONTRACT')
 cloud_meta=[]
 for n,(ident,available,m) in zip(names,chosen):
  field=next(v for v in m.fields if v.name=='timestamp');assert field.datatype==7
  t=np.ndarray((m.width*m.height,),dtype='<f4',buffer=m.data,offset=field.offset,strides=(m.point_step,));assert np.isfinite(t).all()
  cloud_meta.append(dict(topic=n,message_id=ident,header_ns=ns(m.header.stamp),available_ns=available,frame=m.header.frame_id,points=m.width*m.height,point_start_ns=ns(m.header.stamp)+int(np.rint(t.min()*1e9)),point_end_ns=ns(m.header.stamp)+int(np.rint(t.max()*1e9))))
 receipt=dict(stationarity=stats,clouds=cloud_meta,selection='first front after recorder+135s; closest left/right header within10ms',reference_candidates=False,global_attempts=1)
 (out/'input_contract.json').write_text(json.dumps(receipt,indent=2))
 rclpy.init();node=rclpy.create_node('bg_c1_global_init_input');qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
 pubs=[node.create_publisher(PointCloud2,n,qos) for n in names];clock=node.create_publisher(Clock,'/clock',10)
 transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 results={}
 def pose_cb(m):
  q=m.pose.pose.orientation;t=m.pose.pose.position;results['map_pose']=dict(frame=m.header.frame_id,publication_stamp_ns=ns(m.header.stamp),position=[t.x,t.y,t.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],covariance=list(m.pose.covariance))
 def ned_cb(m):results['ned_protocol']=dict(position=[m.position.x,m.position.y,m.position.z],quaternion_xyzw=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w])
 subs=[node.create_subscription(PoseWithCovarianceStamped,'/initialpose3d',pose_cb,transient),node.create_subscription(Bool,'/init_flag',lambda m:results.update(success_flag=m.data),transient),node.create_subscription(Pose,'/lio_init_pose',ned_cb,transient)]
 static=StaticTransformBroadcaster(node);ts=[]
 for e in json.load(open(a.audit))['tf_static']:
  if e['child'] not in {'front_wheel','imu_bosch','lidar_front','lidar_left','lidar_right'}:continue
  t=TransformStamped();t.header.frame_id=e['parent'];t.child_frame_id=e['child'];stamp(t.header.stamp,max(headers));t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=e['translation'];t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=e['quaternion'];ts.append(t)
 static.sendTransform(ts);m=Clock();stamp(m.clock,max(headers));clock.publish(m)
 log=(out/'global_init.log').open('wb');proc=None;reason='UNKNOWN'
 try:
  proc=subprocess.Popen([str(p/'globalinit_build/global_init')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
  start=time.monotonic();sent=False
  while time.monotonic()-start<100:
   rclpy.spin_once(node,timeout_sec=.01);clock.publish(m)
   txt=(out/'global_init.log').read_text(errors='replace')
   if proc.poll() is not None:reason='PROCESS_EXIT';break
   if not sent and 'WAIT_INPUT_READY' in txt and all(node.count_subscribers(n)>0 for n in names) and time.monotonic()-start>3:
    # The native ready barrier has been created. Publish each ORIGINAL message once.
    for pub,item in zip(pubs,chosen):pub.publish(item[2])
    sent=True;receipt['publish_wall_ns']=time.monotonic_ns()
   if results.get('success_flag') and 'map_pose' in results:reason='NATIVE_SUCCESS';break
   if '[GLOBAL_INIT][ROUND]' in txt and 'FAILED reason=' in txt:reason='NATIVE_QUALITY_FAILURE';break
  else:reason='BOUNDED_TIMEOUT'
 finally:
  if proc and proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
  log.close();results.update(reason=reason,source_scan_header_ns=max(headers),source_available_ns=max(v[1] for v in chosen),process_returncode=proc.returncode if proc else None,reference_used=False)
  (out/'result.json').write_text(json.dumps(results,indent=2));(out/'input_contract.json').write_text(json.dumps(receipt,indent=2));node.destroy_node();rclpy.shutdown()
 print(reason,stats)
 if reason!='NATIVE_SUCCESS':raise SystemExit(2)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--bag',required=True);p.add_argument('--motion',required=True);p.add_argument('--audit',required=True);main(p.parse_args())
