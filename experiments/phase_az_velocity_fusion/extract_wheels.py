"""Read-only four-wheel evidence; individual channels never enter this candidate."""
import argparse,sqlite3,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--optical',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
es=json.loads(a.optical.read_text())['events'];lo=min(e['header_ns'] for e in es);hi=max(e['header_ns'] for e in es);c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);tid,typ=c.execute("select id,type from topics where name like '%/wheels_speed_01'").fetchone();cls=get_message(typ);rows=[]
for ns,b in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
 m=deserialize_message(b,cls);rows.append([m.timestamp.nanoseconds,ns,m.wss_speed_fl_rad_s,m.wss_speed_fr_rad_s,m.wss_speed_rl_rad_s,m.wss_speed_rr_rad_s])
r=np.array(rows);v=r[:,2:];mean=v.mean(axis=1);valid=mean>1;ratio=(v[valid,2:].mean(axis=1)-v[valid,:2].mean(axis=1))/mean[valid];out=dict(messages=len(r),all_finite=bool(np.isfinite(r).all()),nonzero_per_channel=(v>0).sum(axis=0).tolist(),front_rear_relative_difference_quantiles=np.percentile(ratio,[0,5,50,95,100]).tolist(),receipt_age_ms_quantiles=np.percentile((r[:,1]-r[:,0])*1e-6,[0,50,95,100]).tolist(),interpretation='decoded four angular-rate fields only; not a calibrated slip label, tyre radius fit or additional estimator input')
np.savez_compressed(a.output.with_suffix('.npz'),raw_wheels=r);a.output.write_text(json.dumps(out,indent=2)+'\n')
