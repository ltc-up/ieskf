"""ONE conditional pit-to-right3 FAR run. One recorded mean, native nine-state EKF.
Independent clock and bounded prefetch/logger. No GNSS/recorded dynamic TF relay.
"""
import argparse,json,os,time,signal,subprocess,threading,hashlib
from pathlib import Path
import numpy as np,yaml
from continuous_inputs import START,INIT,END,stream,Prefetch,AsyncLog,ns
from clock_pump import ClockPump

def stamp(t,n):t.sec=int(n)//10**9;t.nanosec=int(n)%10**9
def main(a):
 import rclpy
 from rclpy.executors import SingleThreadedExecutor
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from sensor_msgs.msg import Imu,PointCloud2,PointField
 from nav_msgs.msg import Odometry
 from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
 from tsl_msgs.msg import TSLDefinition,TSLValues
 from geometry_msgs.msg import TransformStamped
 from tf2_ros import StaticTransformBroadcaster
 root=Path(a.root);out=root/'far_recorded_init';out.mkdir(exist_ok=False)
 assert json.load(open(root/'timing_contract/summary.json'))['reason']=='COMPLETED'
 assert json.load(open(root/'realtime_contract/summary.json'))['reason']=='COMPLETED'
 init=json.load(open(root/'recorded_common_init/initialization.json'));audit=json.load(open(a.audit));Tbi=np.array(json.load(open(a.extrinsics))['T_base_imu']);source=Path(a.source);bg=root.parent
 cfg=yaml.safe_load((source/'config/far-lio.yml').read_text());lp=cfg['/core/state']['LidarOdometry']['ros__parameters'];sp=cfg['/core/state']['StateEstimation']['ros__parameters']
 lp['pipeline']['update_map']=False;lp['pipeline']['debug_mode']=False
 lp['node'].update(odom_frame='map',child_frame='base_link',cloud_frame='base_link',input_pointcloud='/c1/input/fused',input_map=str(bg/'runtime/input_contract/existing_target_xyz.pcd'),update_map_srv='',wait_tf=False)
 lp['model'].update(dict(zip(['initial_pos_x','initial_pos_y','initial_pos_z'],init['mean']['position'])));lp['model'].update(dict(zip(['initial_rot_x','initial_rot_y','initial_rot_z','initial_rot_w'],init['mean']['quaternion_xyzw'])))
 sp['node']['odom_frame_id']='map';sp['inputs']['imus']['primary']['message']['topic']='/c1/input/imu';sp['measurements']['lidar_gicp']['message']['topic']='/c1/ekf/lidar_measurement';sp['measurements']['lidar_gicp']['status']['topic']='/c1/ekf/lidar_status'
 config=out/'far_lio.yml';config.write_text(yaml.safe_dump(cfg,sort_keys=False));install=bg/'runtime/utf8/install';bins=[install/'tam_odometry/lib/tam_odometry/tam_cuda_gicp_ext',install/'tam_state_estimation_node/lib/tam_state_estimation_node/tam_state_estimation_node_ekf_3d_nh']
 feed=Prefetch(stream(a.bag,audit,Tbi));ilog=AsyncLog(out/'inputs.jsonl');records=AsyncLog(out/'records.jsonl');clock=ClockPump(out/'clock.csv',START,END)
 rclpy.init();n=rclpy.create_node('bg_c1_conditional_input');best=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT);offered=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
 cp=n.create_publisher(PointCloud2,'/c1/input/fused',offered);ip=n.create_publisher(Imu,'/c1/input/imu',offered);mp=n.create_publisher(Odometry,'/c1/ekf/lidar_measurement',offered);dp=n.create_publisher(DiagnosticArray,'/c1/ekf/lidar_status',offered);now={'epoch':0};defs={};state_counts={'ekf':0,'lidar':0};first={};failure=[]
 def store(kind,**kw):
  try:records.put(dict(kind=kind,received_wall_ns=time.monotonic_ns(),sim_ns=START+max(0,time.monotonic_ns()-now['epoch']) if now['epoch'] else START,**kw))
  except Exception as e:failure.append(repr(e))
 def odom(m,kind):
  p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear
  row=dict(stamp_ns=ns(m.header.stamp),frame=m.header.frame_id,child=m.child_frame_id,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],velocity=[v.x,v.y,v.z],pose_cov=list(m.pose.covariance),twist_cov=list(m.twist.covariance));state_counts[kind]+=1
  if kind not in first:first[kind]=row
  store(kind,**row)
  if kind=='lidar':mp.publish(m)
 def diag(m):
  store('diagnostic',stamp_ns=ns(m.header.stamp),status=[dict(name=x.name,level=int.from_bytes(x.level,'little') if isinstance(x.level,bytes) else int(x.level),message=x.message) for x in m.status]);dp.publish(m)
 subs=[n.create_subscription(Odometry,'/core/state/lidar_odometry',lambda m:odom(m,'lidar'),best),n.create_subscription(Odometry,'/core/state/odometry',lambda m:odom(m,'ekf'),best),n.create_subscription(DiagnosticArray,'/core/state/lidar_odometry_status',diag,best)]
 types=['bool','int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
 def definition(m,name):defs[(name,m.definition_hash)]={t:list(getattr(m,t+'_names')) for t in types}
 def values(m,name):
  d=defs.get((name,m.definition_hash))
  if d:store('timing_debug',node=name,stamp_ns=ns(m.stamp),values={k:float(v) for t in types for k,v in zip(d[t],getattr(m,t+'_values'))})
 for name in ['LidarOdometry','StateEstimation']:
  topic='/debug/core/state/'+name;subs.extend([n.create_subscription(TSLDefinition,topic+'/def',lambda m,z=name:definition(m,z),best),n.create_subscription(TSLValues,topic,lambda m,z=name:values(m,z),best)])
 ts=[]
 for e in audit['tf_static']:
  if e['child'] not in {'front_wheel','imu_bosch','lidar_front','lidar_left','lidar_right'}:continue
  t=TransformStamped();t.header.frame_id=e['parent'];t.child_frame_id=e['child'];stamp(t.header.stamp,START);t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=e['translation'];t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=e['quaternion'];ts.append(t)
 static=StaticTransformBroadcaster(n);static.sendTransform(ts);ex=SingleThreadedExecutor();ex.add_node(n);th=threading.Thread(target=ex.spin,daemon=True);th.start();processes=[];logs=[];reason='UNKNOWN';sentinit=False;counts={};hasher=hashlib.sha256();lastavail=START;late=[];scored=[x for x in json.load(open(root/'timing_contract/cloud_identity.json'))];first_headers=set();all_headers=[]
 (out/'manifest.json').write_text(json.dumps(dict(config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),binaries={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in bins},source_commit='ef20510316ad1f8dbf2e428e4a193f700493eccd',initial_mean_source_id=init['source']['message_id'],recorded_P_copied=False,online_reference_after_init=False,update_map=False),indent=2))
 try:
  for binary,name in zip(bins,['LidarOdometry','StateEstimation']):
   log=(out/(name+'.log')).open('wb');logs.append(log);processes.append(subprocess.Popen([str(binary),'--ros-args','--params-file',str(config),'-r','__node:='+name,'-r','__ns:=/core/state'],stdout=log,stderr=subprocess.STDOUT))
  time.sleep(4)
  if any(x.poll() is not None for x in processes):raise RuntimeError('NODE_START_FAILURE')
  graph={name:n.get_subscriber_names_and_types_by_node(name,'/core/state') for name in ['LidarOdometry','StateEstimation']};(out/'subscriptions.json').write_text(json.dumps(graph,indent=2))
  if any(any(t in topic.lower() for t in ['gnss','gps','vectornav']) for vv in graph.values() for topic,_ in vv):raise RuntimeError('REFERENCE_TOPIC_IN_GRAPH')
  now['epoch']=epoch=clock.start()
  while True:
   e=feed.get()
   if e['kind']=='end':tail=e;break
   if e['available']<lastavail:raise RuntimeError('AVAILABLE_REVERSAL')
   lastavail=e['available'];due=epoch+e['available']-START
   while time.monotonic_ns()<due:time.sleep(min(.001,max(0,(due-time.monotonic_ns())/1e9)))
   if failure:raise RuntimeError(failure[0])
   if any(x.poll() is not None for x in processes):raise RuntimeError('NODE_EXIT')
   if not sentinit and e['available']>=INIT:
    m=Odometry();stamp(m.header.stamp,INIT);m.header.frame_id='map';m.child_frame_id='base_link';m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=init['mean']['position'];m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=init['mean']['quaternion_xyzw']
    # All-zero unused message covariance is NOT historical P or a .01 replacement.
    # Native set_initial_state sets x only; native constructor retains zero P.
    d=DiagnosticArray();stamp(d.header.stamp,INIT);s=DiagnosticStatus();s.name='authorized_one_time_initial_mean';s.level=DiagnosticStatus.OK;s.message='authorized recorded mean; verified stationary zero velocity';d.status=[s];dp.publish(d);mp.publish(m);sentinit=True;store('one_time_common_initial_mean',source_message_id=init['source']['message_id'],stamp_ns=INIT)
   tick=time.monotonic_ns()
   if e['kind']=='cloud':
    identities=e['meta']['raw_identities'];all_headers.append(e['header'])
    if e['available']>=INIT:
     f=e['data'];m=PointCloud2();stamp(m.header.stamp,e['header']);m.header.frame_id='base_link';m.height=1;m.width=len(f);m.point_step=f.dtype.itemsize;m.row_step=m.width*m.point_step;m.is_dense=True;m.fields=[PointField(name=k,offset=f.dtype.fields[k][1],datatype=7 if k!='sensor_id' else 2,count=1) for k in f.dtype.names];m.data=f.tobytes();cp.publish(m);first_headers.add(e['header'])
   else:
    identities=[dict(id=e['id'],cdr_sha256=e['sha'])]
    if e['kind']=='imu' and e['adapted'] is not None:
     z=e['adapted'];m=Imu();stamp(m.header.stamp,e['header']);m.header.frame_id='base_link';m.orientation_covariance[0]=-1.;m.angular_velocity_covariance[0]=-1.;m.linear_acceleration_covariance[0]=-1.;m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,z['omega_base']);m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,z['force_base']);ip.publish(m)
   hasher.update(json.dumps(identities,sort_keys=True).encode());counts[e['kind']]=counts.get(e['kind'],0)+1;finish=time.monotonic_ns();lateness=(finish-due)/1e6;late.append(lateness);ilog.put(dict(kind=e['kind'],header=e['header'],available=e['available'],publish_wall_ns=finish,publish_sim_ns=START+finish-epoch,identities=identities,meta=e.get('meta'),dispatch_ms=(finish-tick)/1e6,lateness_ms=lateness,pre_initialization=e['available']<INIT))
   if lateness>100:raise RuntimeError('SEVERE_INPUT_DELIVERY_DELAY')
   if e['available']>INIT+3_000_000_000 and state_counts['ekf']==0:raise RuntimeError('NATIVE_EKF_INITIALIZATION_FAILED')
   if int((e['available']-START)/1e9)%5==0 and sum(x.stat().st_size for x in out.iterdir() if x.is_file())>512*1024**2:raise RuntimeError('RESULT_RESOURCE_CAP')
  if hasher.hexdigest()!=json.load(open(root/'timing_contract/summary.json'))['ordered_identity_sha256']:raise RuntimeError('INPUT_IDENTITY_MISMATCH')
  # Do not advance past the frozen endpoint to manufacture extra future-state output.
  time.sleep(.15);reason='COMPLETED'
 except Exception as err:reason=repr(err)
 finally:
  feed.close()
  for p in processes:
   if p.poll() is None:p.send_signal(signal.SIGINT)
  for p in processes:
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  cs=clock.close();ex.shutdown();th.join(3);n.destroy_node();rclpy.shutdown();ilog.close();records.close()
  for f in logs:f.close()
  summary=dict(reason=reason,counts=counts,state_counts=state_counts,clock=cs,first_outputs=first,initial_mean_submitted_once=sentinit,original_P_not_copied=True,ordered_identity_sha256=hasher.hexdigest(),prefetch_peak_bytes=feed.peak,max_delivery_delay_ms=max(late,default=0),process_returncodes=[p.returncode for p in processes])
  (out/'run_status.json').write_text(json.dumps(summary,indent=2));print(reason,counts,state_counts)
 if reason!='COMPLETED':raise SystemExit(2)
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','bag','audit','extrinsics','source']];main(p.parse_args())
