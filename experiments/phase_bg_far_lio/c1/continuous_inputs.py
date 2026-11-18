"""Bounded read-only three-LiDAR/IMU/wheel stream; no navigation reference reader.
Prefetch may read future bytes; release remains governed by recorder availability.
"""
import hashlib,json,sqlite3,time,queue,threading
from pathlib import Path
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from input_adapter import decode_cloud,fuse_three,adapt_imu
ORIGIN=1763205419863639180
START=ORIGIN+130_000_000_000
INIT=ORIGIN+135_000_000_000
END=1763205765669702260
NAMES=['/sensor/lidar_front/points','/sensor/lidar_left/points','/sensor/lidar_right/points']
def ns(t):return int(t.sec)*10**9+int(t.nanosec)
def stream(bag,audit,Tbi):
 from rclpy.serialization import deserialize_message
 from sensor_msgs.msg import PointCloud2,Imu
 from geometry_msgs.msg import TwistWithCovarianceStamped
 from scipy.spatial.transform import Rotation
 from collections import deque
 db=sqlite3.connect('file:'+str(bag)+'?mode=ro&immutable=1',uri=True)
 names={i:n for i,n in db.execute('select id,name from topics')};want=NAMES+['/sensor/bosch/imu','/localization/wheel_velocity'];ids=[i for i,n in names.items() if n in want]
 transforms={}
 # Resolve static chain without consulting any recorded dynamic TF.
 edges={e['child']:e for e in audit['tf_static']}
 def tf(child):
  if child=='base_link':return np.eye(4)
  e=edges[child];z=np.eye(4);z[:3,:3]=Rotation.from_quat(e['quaternion']).as_matrix();z[:3,3]=e['translation'];return tf(e['parent'])@z
 for n in NAMES:transforms[n]=tf(n.split('/')[2])
 pending={n:deque() for n in NAMES};prev=None;seen=set();lastimu=None;lastavailable=START;rawcount=0
 query='select id,topic_id,timestamp,data from messages where timestamp>=? and timestamp<=? and topic_id in ('+','.join('?' for _ in ids)+') order by timestamp,id'
 for ident,tid,available,blob in db.execute(query,[START,END,*ids]):
  tick=time.monotonic_ns();name=names[tid];sha=hashlib.sha256(blob).hexdigest()
  if available<lastavailable:raise ValueError('AVAILABLE_TIME_REVERSAL')
  lastavailable=available
  if name in NAMES:
   m=deserialize_message(blob,PointCloud2);h=ns(m.header.stamp);sid=NAMES.index(name)
   fields={f.name:dict(offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields}
   c=decode_cloud(m.data,fields,m.width*m.height,m.point_step,h,available,sid,ident,transforms[name],m.is_bigendian);pending[name].append((c,sha));rawcount+=1
   if max(len(v) for v in pending.values())>2:raise ValueError('UNPAIRED_LIDAR_INPUT')
   if all(pending.values()):
    cc=[pending[n][0] for n in NAMES];headers=[v[0].header_ns for v in cc]
    if max(headers)-min(headers)>10_000_000:raise ValueError('THREE_LIDAR_SYNC_FAILURE')
    fused,meta=fuse_three([v[0] for v in cc]);meta.update(raw_headers_ns=headers,point_start_ns=min(int(c.header_ns+np.rint(c.offset_s.astype(float)*1e9).min()) for c,_ in cc),raw_identities=[dict(id=c.identity,cdr_sha256=s) for c,s in cc],source_n=len(fused),prepare_ms=(time.monotonic_ns()-tick)/1e6)
    for n in NAMES:pending[n].popleft()
    yield dict(kind='cloud',header=meta['header_ns'],available=meta['available_ns'],meta=meta,data=fused)
  elif name=='/sensor/bosch/imu':
   m=deserialize_message(blob,Imu);h=ns(m.header.stamp);w=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);f=np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
   if lastimu is not None and not 0<h-lastimu<=25_000_000:raise ValueError('ORIGINAL_IMU_GAP_OR_REVERSAL')
   lastimu=h
   if prev is None:
    prev=dict(stamp_ns=h,omega_base=Tbi[:3,:3]@w)
    # First raw IMU is delivered to15D/contract checker; FAR needs past gyro for lever alpha.
    adapted=None
   else:adapted=adapt_imu(w,f,Tbi,h,prev);prev=adapted
   yield dict(kind='imu',header=h,available=available,id=ident,sha=sha,omega=(Tbi[:3,:3]@w).tolist(),force=(Tbi[:3,:3]@f).tolist(),adapted=adapted,prepare_ms=(time.monotonic_ns()-tick)/1e6)
  else:
   m=deserialize_message(blob,TwistWithCovarianceStamped);h=ns(m.header.stamp)
   yield dict(kind='wheel',header=h,available=available,id=ident,sha=sha,speed=m.twist.twist.linear.x,variance=m.twist.covariance[0],prepare_ms=(time.monotonic_ns()-tick)/1e6)
 yield dict(kind='end',header=END,available=END,pending_tail={n:[c.identity for c,_ in v] for n,v in pending.items()},raw_count=rawcount)
 db.close()
class Prefetch:
 """Single producer, at most512 queued events and128MiB payload; fail on producer error."""
 def __init__(self,it):
  self.q=queue.Queue(512);self.cv=threading.Condition();self.bytes=0;self.peak=0;self.maxbytes=128*1024**2;self.stop=False
  def run():
   try:
    for e in it:
     size=e['data'].nbytes if 'data'in e else 4096
     with self.cv:
      self.cv.wait_for(lambda:self.stop or self.bytes+size<=self.maxbytes)
      if self.stop:return
      self.bytes+=size;self.peak=max(self.peak,self.bytes)
     self.q.put((e,size))
   except Exception as ex:self.q.put((dict(kind='error',reason=repr(ex)),0))
  self.thread=threading.Thread(target=run,daemon=True);self.thread.start()
 def get(self):
  e,size=self.q.get(timeout=15)
  with self.cv:self.bytes-=size;self.cv.notify_all()
  if e['kind']=='error':raise RuntimeError(e['reason'])
  return e
 def close(self):
  with self.cv:self.stop=True;self.cv.notify_all()
class AsyncLog:
 def __init__(self,path):
  self.q=queue.Queue(1024);self.error=None;self.count=0
  def run():
   try:
    with Path(path).open('x',buffering=1<<20) as f:
     while True:
      row=self.q.get()
      if row is None:break
      f.write(json.dumps(row,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())+'\n');self.count+=1
   except Exception as e:self.error=repr(e)
  self.thread=threading.Thread(target=run,daemon=True);self.thread.start()
 def put(self,row):
  if self.error:raise RuntimeError(self.error)
  try:self.q.put_nowait(row)
  except queue.Full:raise RuntimeError('ASYNC_LOG_OVERFLOW')
 def close(self):
  self.q.put(None,timeout=5);self.thread.join(timeout=5)
  if self.thread.is_alive() or self.error:raise RuntimeError('ASYNC_LOG_INCOMPLETE')
def audit_stream(args):
 p=Path(args.root);out=p/'timing_contract';out.mkdir(exist_ok=False)
 audit=json.load(open(args.audit));Tbi=np.array(json.load(open(args.extrinsics))['T_base_imu'])
 inp=Prefetch(stream(args.bag,audit,Tbi));log=AsyncLog(out/'input_ledger.jsonl');counts={};h=hashlib.sha256();prev=START;clouds=[];reason='COMPLETED'
 # Full-span content/identity scan, with no ROS or estimator. Separate realtime scheduler test follows.
 try:
  while True:
   before=time.monotonic_ns();e=inp.get();after=time.monotonic_ns()
   if e['kind']=='end':tail=e;break
   if e['available']<prev:raise ValueError('AVAILABLE_REVERSAL')
   prev=e['available'];counts[e['kind']]=counts.get(e['kind'],0)+1
   if e['kind']=='cloud':
    identity=e['meta']['raw_identities'];clouds.append(dict(header=e['header'],available=e['available'],ids=[x['id'] for x in identity]));assert e['meta']['max_point_time_roundtrip_error_ns']<=4
   else:identity=[dict(id=e['id'],cdr_sha256=e['sha'])]
   h.update(json.dumps(identity,sort_keys=True).encode());log.put(dict(kind=e['kind'],header=e['header'],available=e['available'],identities=identity,queue_wait_ms=(after-before)/1e6,prepare_ms=e.get('prepare_ms',e.get('meta',{}).get('prepare_ms')),meta=e.get('meta')))
 except Exception as ex:reason=repr(ex);tail={}
 finally:inp.close();log.close()
 summary=dict(reason=reason,counts=counts,ordered_identity_sha256=h.hexdigest(),prefetch_peak_bytes=inp.peak,tail=tail,localization=False,realtime_certification=False)
 (out/'cloud_identity.json').write_text(json.dumps(clouds));(out/'summary.json').write_text(json.dumps(summary,indent=2));print(summary)
 if reason!='COMPLETED':raise SystemExit(2)
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();[a.add_argument('--'+n,required=True) for n in ['root','bag','audit','extrinsics']];audit_stream(a.parse_args())
