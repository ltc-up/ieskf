"""Read-only topic/field/time and auxiliary dynamics inventory; no fusion."""
import argparse,sqlite3,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
jobs=json.loads(a.bindings.read_text());lo=min(j['native_reference_ns'] for j in jobs);hi=max(j['native_reference_ns'] for j in jobs);c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);out={};series={}
for tid,name,typ in c.execute('select id,name,type from topics').fetchall():
 if not (name in ['/sensor/bosch/imu','/sensor/kistler/imu','/vectornav/raw/imu','/imu/vectornav/data'] or name.startswith('/sensor/radar_')):continue
 cls=get_message(typ);count=0;st=[];acc=[];gyro=[];frames=set();fields=None
 for recv,blob in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
  m=deserialize_message(blob,cls);count+=1
  if hasattr(m,'fields'):
   fields=[f.name for f in m.fields];break
  ts=m.header.stamp.sec*10**9+m.header.stamp.nanosec;st.append(ts);frames.add(m.header.frame_id)
  f=getattr(m,'linear_acceleration',getattr(m,'uncompaccel',None));w=getattr(m,'angular_velocity',getattr(m,'uncompgyro',None))
  if f is not None:acc.append([f.x,f.y,f.z])
  if w is not None:gyro.append([w.x,w.y,w.z])
 out[name]=dict(type=typ,scanned=count,frames=sorted(frames),point_fields=fields,header_gap_ms_quantiles=np.percentile(np.diff(st)*np.array(1e-6),[0,50,95,100]).tolist() if len(st)>1 else None,accel_mean=np.mean(acc,axis=0).tolist() if acc else None)
 if st:series[name+'_header']=st;series[name+'_accel']=acc;series[name+'_gyro']=gyro
np.savez_compressed(a.output.with_suffix('.npz'),**series);a.output.write_text(json.dumps(out,indent=2)+'\n')
