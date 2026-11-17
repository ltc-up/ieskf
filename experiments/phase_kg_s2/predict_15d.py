"""Existing15D propagation-only comparator from identical saved initial mean/fullP.
No LiDAR observation, no reference reader, no production state write.
"""
import sys,json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'src'))
from geometry_ieskf.state import State
from geometry_ieskf.inertial import InertialPredictor
from ndt_global.delayed_local import DelayedLocalReplay
root=Path(sys.argv[1]);cfg=json.load(open(root/'inputs/series.json'));Tbi=np.array(cfg['T_base_imu']);lever=Tbi[:3,3];Rb=Tbi[:3,:3];motion=json.load(open(cfg['motion']))['events'];sensor=[e for e in motion if e['kind']in('imu','wheel')and e.get('valid')];imus=[e for e in sensor if e['kind']=='imu'];dest=root/'reference_15d_predictions';dest.mkdir(exist_ok=False)
for series in cfg['series']:
 if series['id']=='pass713':continue
 rows=series['records'];t0=rows[0]['t'];t1=rows[-1]['t'];pr=rows[0]['prior'];x=State(np.array(pr['p']),np.array(pr['v']),np.array(pr['R']),np.array(pr['bg']),np.array(pr['ba']),t0);P=np.array(pr['P']);im=max((e for e in imus if e['header_ns']<=t0),key=lambda e:e['header_ns']);held=dict(header_ns=im['header_ns'],omega=Rb@im['omega'],force=Rb@im['force']);replay=DelayedLocalReplay(x,P,held,lever);events=sorted([e for e in sensor if t0<e['header_ns']<=t1],key=lambda e:(e['recorder_ns'],e['message_id']));i=0;out=[]
 for r in rows:
  t=r['t'];available=r['available']
  while i<len(events)and events[i]['header_ns']<=t and events[i]['recorder_ns']<=available:
   e=events[i].copy()
   if e['kind']=='imu':e['omega']=Rb@e['omega'];e['force']=Rb@e['force']
   replay.ingest(e);i+=1
  y,Q,h=replay.current()
  if y.t<t:y,Q=InertialPredictor().predict(y,Q,h['omega'],h['force'],(t-y.t)*1e-9);y.t=t
  assert y.t==t
  st=dict(T_map_base_R=y.R.tolist(),p_base=(y.p-y.R@lever).reshape(3,1).tolist(),bias_accel_imu=(Rb.T@y.ba).reshape(3,1).tolist(),bias_gyro_imu=(Rb.T@y.bg).reshape(3,1).tolist(),velocity_map_imu=y.v.reshape(3,1).tolist());out.append(dict(t=t,initial=st,final=st,total_step_ms=0,update_ms=0,diagnostic_only=dict(P_15D=Q.tolist())))
 sd=dest/(series['id']+'_E0');sd.mkdir();(sd/'steps.jsonl').write_text(''.join(json.dumps(x)+'\n'for x in out))
print('existing15D propagation-only; timing intentionally NOT MEASURED')
