"""Read-only sensor extraction; no GNSS/pose data or fitted constants."""
import argparse,sqlite3,json,collections
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);a=p.parse_args()
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
jobs=json.loads(a.bindings.read_text());lo=min(j['native_reference_ns'] for j in jobs)-1_000_000_000;hi=max(j['native_reference_ns'] for j in jobs)+1_000_000_000;c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);wanted=['/sensor/kistler/measurement','/sensor/kistler/status','/flyeagle/a2rl/eav25_bsu/kistler_correvit'];topics={n:(i,t) for i,n,t in c.execute('select id,name,type from topics') if n in wanted};events={}
for name,(tid,typ) in topics.items():
 cls=get_message(typ);rows=[]
 for mid,recv,blob in c.execute('select id,timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
  m=deserialize_message(blob,cls)
  if name.endswith('/measurement'):r=dict(header_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,message_id=mid,recorder_ns=recv,frame=m.header.frame_id,raw=[m.velocity_raw.x,m.velocity_raw.y],corrected=[m.velocity.x,m.velocity.y],raw_resultant=m.resultant_velocity_raw.x)
  elif name.endswith('/status'):r=dict(header_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,message_id=mid,recorder_ns=recv,stst=m.stst,head_status=m.head_status,direction_head_is_valid=m.direction_head_is_valid,temperature_ok=m.temperature_ok,filter_setting=m.filter_setting)
  else:r=dict(recorder_ns=recv,message_id=mid,stamp=str(m.timestamp),x=m.vel_x_cor,y=m.vel_y_cor)
  rows.append(r)
 events[name]=rows
raw=events['/sensor/kistler/measurement'];status=events['/sensor/kistler/status'];selected=[];status_cursor=0;last_status=None;last_bin=None;invalid=collections.Counter()
for r in raw:
 while status_cursor<len(status) and status[status_cursor]['recorder_ns']<=r['recorder_ns']:
  last_status=status[status_cursor];status_cursor+=1
 binid=r['header_ns']//10_000_000
 if binid==last_bin:continue
 last_bin=binid
 valid=last_status is not None and last_status['stst']==0 and 0<=r['header_ns']-last_status['header_ns']<=250_000_000 and r['frame']=='kistler' and np.isfinite(r['raw']).all()
 selected.append(dict(kind='optical',header_ns=r['header_ns'],recorder_ns=r['recorder_ns'],message_id=r['message_id'],velocity=r['raw'],status_good=bool(valid),variance=[[.26,0],[0,.26]],status=last_status))
a.output.write_text(json.dumps(dict(events=selected,raw_topics=events),separators=(',',':')))
def qs(v):return np.percentile(v,[0,50,95,100]).tolist()
v=np.array([r['raw'] for r in raw]);out=dict(raw_messages=len(raw),events=len(selected),good=sum(e['status_good'] for e in selected),frame_counts=dict(collections.Counter(r['frame'] for r in raw)),status_stst=dict(collections.Counter(r['stst'] for r in status)),head_status=dict(collections.Counter(r['head_status'] for r in status)),status_other={k:dict(collections.Counter(r[k] for r in status)) for k in ['direction_head_is_valid','temperature_ok','filter_setting']},raw_x_quantiles=qs(v[:,0]),raw_y_quantiles=qs(v[:,1]),exact_adjacent_raw_repeats=int(np.all(np.diff(v,axis=0)==0,axis=1).sum()),header_gap_ms=qs(np.diff([r['header_ns'] for r in raw])*1e-6),arrival_minus_header_ms=qs([(r['recorder_ns']-r['header_ns'])*1e-6 for r in raw]),raw_CAN_messages=len(events[wanted[2]]),selected_time_rule='first available packet per fixed10ms Header bin; past recorded status only; no future reference',variance_provenance='existing0.01 publisher engineering variance plus frozen0.5m/s slip envelope, uncalibrated',GNSS_used=False);a.audit.write_text(json.dumps(out,indent=2)+'\n');print(out)
