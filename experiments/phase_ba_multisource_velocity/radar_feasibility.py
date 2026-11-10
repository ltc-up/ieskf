"""Read-only Doppler support/ambiguity audit, no source-estimate fusion."""
import argparse,sqlite3,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
jobs=json.loads(a.bindings.read_text());lo=min(j['native_reference_ns'] for j in jobs);hi=lo+20_000_000_000;c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);out={};series={}
for tid,name,typ in c.execute("select id,name,type from topics where name like '/sensor/radar_%/points'").fetchall():
 cls=get_message(typ);vs=[];ints=[];counts=[];heights=[];records=[]
 for rec,blob in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
  m=deserialize_message(blob,cls);fields={f.name:f for f in m.fields};dtype=np.dtype(dict(names=list(fields),formats=[('>' if m.is_bigendian else '<')+{7:'f4',8:'f8',6:'u4',5:'i4',4:'u2',3:'i2',2:'u1',1:'i1'}[f.datatype] for f in fields.values()],offsets=[f.offset for f in fields.values()],itemsize=m.point_step));d=np.frombuffer(m.data,dtype=dtype);v=np.array(d['velocity']);vv=np.array(d['velocity_interval']);xyz=np.c_[d['x'],d['y'],d['z']];finite=np.isfinite(xyz).all(axis=1)&np.isfinite(v)&np.isfinite(vv);xyz=xyz[finite];v=v[finite];vv=vv[finite];vs.extend(v);ints.extend(vv);counts.append(len(v));heights.extend(xyz[:,2]);records.append(dict(header_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,recorder_ns=rec,frame=m.header.frame_id,xyz=xyz.tolist(),velocity=v.tolist(),interval=vv.tolist()))
 out[name]=dict(frames=len(counts),points=len(vs),frame_points_quantiles=np.percentile(counts,[0,50,95,100]).tolist(),radial_velocity_quantiles=np.percentile(vs,[0,5,50,95,100]).tolist(),unambiguous_interval_quantiles=np.percentile(ints,[0,50,95,100]).tolist(),velocity_units='m/s per driver output contract; unresolved periodic aliases must not be silently unwrapped',extrinsic_and_static_target_qualification='NOT_VALIDATED')
 series[name]=records
a.output.write_text(json.dumps(out,indent=2)+'\n');a.output.with_suffix('.private.json').write_text(json.dumps(series,separators=(',',':')))
