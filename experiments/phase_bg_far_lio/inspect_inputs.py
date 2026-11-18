"""Bounded read-only bag inspection; no ROS graph, publishers, replay or algorithms.
Read a single one-second slice and retain metadata/statistics, never point payloads.
"""
import argparse,json,sqlite3,os,hashlib
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
from tf2_msgs.msg import TFMessage

def main(a):
 p=Path(a.bag);db=sqlite3.connect('file:'+str(p)+'?mode=ro&immutable=1',uri=True);topics={r[0]:dict(name=r[1],type=r[2]) for r in db.execute('select id,name,type from topics')};start=db.execute('select min(timestamp) from messages').fetchone()[0]
 relevant={i:t for i,t in topics.items() if t['name'] in ['/sensor/lidar_front/points','/sensor/lidar_left/points','/sensor/lidar_right/points','/sensor/lidar_fusion/points','/sensor/bosch/imu','/sensor/kistler/imu','/tf','/tf_static']}
 result=dict(bag_size=p.stat().st_size,metadata_sha256=hashlib.sha256((p.parent/'metadata.yaml').read_bytes()).hexdigest(),bag_hashed=False,topics=relevant,all_pointcloud_topics=[t for t in topics.values() if t['type']=='sensor_msgs/msg/PointCloud2'],window_seconds=1.,counts={},clouds=[],imus={},tf_dynamic=[],tf_static=[],mode='READ_ONLY_NO_REPLAY')
 imu={};headers={};placeholders=','.join(['?']*len(relevant))
 query=f'select id,topic_id,timestamp,data from messages where timestamp>=? and timestamp<? and topic_id in ({placeholders}) order by timestamp'
 for ident,tid,available,payload in db.execute(query,[start,start+1_000_000_000,*relevant]):
  name=topics[tid]['name'];result['counts'][name]=result['counts'].get(name,0)+1
  if topics[tid]['type']=='sensor_msgs/msg/PointCloud2':
   msg=deserialize_message(payload,PointCloud2);header=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;headers.setdefault(name,[]).append(header)
   if sum(x['topic']==name for x in result['clouds'])>=2:continue
   fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields];item=dict(topic=name,frame=msg.header.frame_id,header_ns=header,available_minus_header_ms=(available-header)/1e6,width=msg.width,height=msg.height,point_step=msg.point_step,row_step=msg.row_step,fields=fields,big_endian=msg.is_bigendian)
   for f in msg.fields:
    if f.name in ['time','t','timestamp','offset_time']:
     dtype={6:'u4',7:'f4',8:'f8',5:'i4'}.get(f.datatype)
     if dtype:
      v=np.ndarray((msg.width*msg.height,),dtype=('<' if not msg.is_bigendian else '>')+dtype,buffer=msg.data,offset=f.offset,strides=(msg.point_step,));item['point_time_raw']=dict(field=f.name,datatype=f.datatype,min=float(np.min(v)),max=float(np.max(v)),median=float(np.median(v)),finite=bool(np.isfinite(v).all()))
   result['clouds'].append(item)
  elif topics[tid]['type']=='sensor_msgs/msg/Imu':
   m=deserialize_message(payload,Imu);h=m.header.stamp.sec*10**9+m.header.stamp.nanosec;imu.setdefault(name,[]).append([h,available,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]);result['imus'][name]=dict(frame=m.header.frame_id,orientation_cov0=m.orientation_covariance[0])
  else:
   m=deserialize_message(payload,TFMessage)
   for t in m.transforms:
    x=dict(parent=t.header.frame_id,child=t.child_frame_id,translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],quaternion=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]);key='tf_static' if name=='/tf_static' else 'tf_dynamic'
    if x not in result[key]:result[key].append(x)
 for name,seq in imu.items():
  v=np.array(seq);dt=np.diff(v[:,0])*1e-6;result['imus'][name].update(samples=len(v),header_dt_ms=dict(min=float(dt.min()),median=float(np.median(dt)),max=float(dt.max())),gyro_mean=v[:,2:5].mean(0).tolist(),force_mean=v[:,5:8].mean(0).tolist(),force_norm_median=float(np.median(np.linalg.norm(v[:,5:8],axis=1))),available_minus_header_ms_median=float(np.median((v[:,1]-v[:,0])*1e-6)))
 result['cloud_intervals']={name:dict(samples=len(v),median_dt_ms=float(np.median(np.diff(v))*1e-6)) for name,v in headers.items() if len(v)>1}
 Path(a.out).write_text(json.dumps(result,indent=2)+'\n');print('Read-only one-second inspection complete',result['counts'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--out',required=True);main(p.parse_args())
