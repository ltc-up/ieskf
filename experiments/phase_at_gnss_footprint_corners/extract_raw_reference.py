"""Read-only targeted raw GNSS/attitude extraction; output must remain private."""
import argparse,json,sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('output');a=p.parse_args();c=sqlite3.connect('file:'+a.bag+'?mode=ro',uri=True);out={}
for tid,name,typ in c.execute('select id,name,type from topics'):
 if name not in ['/vectornav/raw/gps','/vectornav/raw/gps2','/vectornav/raw/attitude','/tf_static']:continue
 rows=[];cls=get_message(typ)
 for receipt,blob in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
  m=deserialize_message(blob,cls)
  if name=='/tf_static':
   rows.extend(dict(parent=z.header.frame_id,child=z.child_frame_id,p=[z.transform.translation.x,z.transform.translation.y,z.transform.translation.z],q=[z.transform.rotation.x,z.transform.rotation.y,z.transform.rotation.z,z.transform.rotation.w]) for z in m.transforms);continue
  r=dict(header=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,receipt=receipt*1e-9)
  if hasattr(m,'fix'):r.update(fix=int(m.fix),lla=[m.poslla.x,m.poslla.y,m.poslla.z],time= {k: str(getattr(m,k)) for k in ['tow','week','timeinfo'] if hasattr(m,k)})
  else:r['q']=[m.quaternion.x,m.quaternion.y,m.quaternion.z,m.quaternion.w]
  rows.append(r)
 out[name]=rows
open(a.output,'w').write(json.dumps(out))
