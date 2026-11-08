"""Build new raw triplets from original bag; never use a recorded motion trajectory.
Window is fixed by a pre-existing normal anchor or prior lap artifact, not outputs.
"""
import argparse,json,sqlite3,hashlib
from pathlib import Path
import numpy as np
from ndt_global.validation import static_transform

def main(a):
 from sensor_msgs.msg import PointCloud2
 from rclpy.serialization import deserialize_message
 base=json.loads(a.anchor.read_text());j0=min((j for j in base if j['window']=='normal'),key=lambda j:j['native_reference_ns']);lo=j0['clouds'][0]['header_ns'];hi=lo+round(a.duration*1e9)
 if a.lap:
  v=dict(line.strip().split('=',1) for line in a.lap.read_text().splitlines() if '=' in line);lo=round(float(v['LAP_START'])*1e9);hi=round(float(v['LAP_END'])*1e9)
 transforms=json.loads(a.input.read_text())['transforms'];con=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);a.private.mkdir(parents=True,exist_ok=True);records={};total=0
 for name in ('front','left','right'):
  topic='/sensor/lidar_'+name+'/points';tid=con.execute('select id from topics where name=?',(topic,)).fetchone()[0];rs=[]
  for mid,recv,blob in con.execute('select id,timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo-500_000_000,hi+500_000_000)):
   m=deserialize_message(blob,PointCloud2);t=m.header.stamp.sec*10**9+m.header.stamp.nanosec
   if not lo-100_000_000<=t<=hi+100_000_000:continue
   fields={f.name:dict(offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields};assert fields['timestamp']['datatype']==7;assert not m.is_bigendian;raw=bytes(m.data);pt=np.ndarray((m.width*m.height,),dtype='<f4',buffer=raw,offset=fields['timestamp']['offset'],strides=(m.point_step,));assert np.isfinite(pt).all();start=t+round(float(pt.min())*1e9);end=t+round(float(pt.max())*1e9);assert end-start<=100_000_000;p=a.private/(str(mid)+'.raw');p.write_bytes(raw);rs.append(dict(header_ns=t,arrival_ns=recv,topic=topic,frame=m.header.frame_id,width=m.width,height=m.height,point_step=m.point_step,row_step=m.row_step,bigendian=m.is_bigendian,fields=fields,raw_path=str(p),raw_sha=hashlib.sha256(raw).hexdigest(),message_id=mid,cdr_sha=hashlib.sha256(blob).hexdigest(),scan_start_ns=start,scan_end_ns=end,T_body_lidar=static_transform(transforms,'base_link',m.header.frame_id).tolist()));total+=1
  records[name]=sorted(rs,key=lambda x:x['header_ns'])
 jobs=[];rejected=0
 for f in records['front']:
  if not lo<=f['header_ns']<hi:continue
  clouds=[f]
  for name in ('left','right'):
   best=min(records[name],key=lambda x:abs(x['header_ns']-f['header_ns']))
   if abs(best['header_ns']-f['header_ns'])>20_000_000:break
   clouds.append(best)
  if len(clouds)!=3:rejected+=1;continue
  ref=max(c['scan_end_ns'] for c in clouds);jobs.append(dict(header=ref,window=a.window,scene=a.window,native_reference_ns=ref,reference_ns=ref,clouds=clouds,gates_pass=True))
 a.output.write_text(json.dumps(jobs));print(json.dumps(dict(raw_messages=total,triplets=len(jobs),rejected_triplets=rejected,duration_s=(hi-lo)/1e9,window=a.window)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--input',type=Path,required=True);p.add_argument('--anchor',type=Path,required=True);p.add_argument('--duration',type=float,default=20);p.add_argument('--lap',type=Path);p.add_argument('--window',default='normal_continuous');p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
