"""Independent logged command/pressure context for the IMU decel proxy."""
import argparse,sqlite3,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--root',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);a=p.parse_args()
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
jobs=json.loads(a.bindings.read_text());lo=min(j['native_reference_ns'] for j in jobs);hi=max(j['native_reference_ns'] for j in jobs);c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);tid,typ=c.execute("select id,type from topics where name='/flyeagle/a2rl/controller/brake'").fetchone();cls=get_message(typ);rows=[]
for rec,b in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
 try:m=deserialize_message(b,cls)
 except Exception as err:
  (a.root/'brake_command_audit.json').write_text(json.dumps(dict(qualification='NOT_VALIDATED_SCHEMA_MISMATCH',topic_type=typ,reason=str(err),pressure_based_braking_label=False,interpretation='installed current message cannot decode historical payload; no partial guessed fields used'),indent=2)+'\n');raise
 rows.append([m.timestamp.nanoseconds,rec,m.front_raw,m.rear_raw,m.act_brake_pressure_fl,m.act_brake_pressure_fr,m.act_brake_pressure_rl,m.act_brake_pressure_rr,int(m.brakewarmup_enabled)])
r=np.array(rows);last={}
for line in (a.root/'B_multilap/optical_updates.jsonl').open():e=json.loads(line);last[e['message_id']]=e
pairs=[]
for e in last.values():
 j=np.searchsorted(r[:,0],e['header_ns'],side='right')-1
 if j<0 or e['header_ns']-r[j,0]>100_000_000:continue
 pairs.append([e['update'].get('own_body_acceleration_x',np.nan),r[j,2],r[j,3],max(r[j,4:8]),r[j,8]])
v=np.array(pairs);decel=v[:,0]<-1;cmd=np.maximum(v[:,1],v[:,2])>1e-3;act=v[:,3]>1e-3;out=dict(messages=len(r),paired=len(v),front_raw_quantiles=np.percentile(r[:,2],[0,50,95,100]).tolist(),rear_raw_quantiles=np.percentile(r[:,3],[0,50,95,100]).tolist(),act_pressure_max_quantiles=np.percentile(r[:,4:8].max(axis=1),[0,50,95,100]).tolist(),positive_command_fraction_when_decelerating=float(np.mean(cmd[decel])),positive_command_fraction_other=float(np.mean(cmd[~decel])),positive_recorded_pressure_fraction_when_decelerating=float(np.mean(act[decel])),warmup_fraction=float(np.mean(v[:,4])),qualification='logged signal nonzero1e-3 only; not calibrated brake force or perfect actuator-state label; no estimator input')
np.savez_compressed(a.root/'brake_command.private.npz',rows=r,paired=v);(a.root/'brake_command_audit.json').write_text(json.dumps(out,indent=2)+'\n')
