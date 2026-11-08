"""Same real sensor prefix in Header, recorder arrival, checkpoint replay order."""
import argparse,json,collections,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State,difference
from ndt_global.validation import static_transform,percentiles
from ndt_global.delayed_local import DelayedLocalReplay

def run(a):
 d=json.loads(a.input.read_text());rows=np.load(a.local/'local_trace.npy',mmap_mode='r');ts=np.load(a.local/'local_stamps_ns.npy');T=static_transform(d['transforms'],'base_link','imu_bosch');Rb=T[:3,:3];lever=T[:3,3];r=rows[0];x=State(r[1:4].copy(),r[4:7].copy(),Rotation.from_quat(r[7:11]).as_matrix(),r[11:14].copy(),r[14:17].copy(),int(ts[0]));P=r[17:].reshape(15,15).copy();raw=max((e for e in d['events'] if e['kind']=='imu' and e['header_ns']<=x.t),key=lambda e:e['header_ns']);imu=dict(header_ns=raw['header_ns'],omega=Rb@raw['omega'],force=Rb@raw['force']);es=[]
 for e in d['events']:
  if e['kind'] not in ('imu','wheel') or not ts[0]<e['header_ns']<=ts[-1]:continue
  z=dict(e)
  if z['kind']=='imu':z['omega']=(Rb@z['omega']).tolist();z['force']=(Rb@z['force']).tolist()
  es.append(z)
 ordered=sorted(es,key=DelayedLocalReplay.key);arrived=sorted(es,key=lambda e:(e['recorder_ns'],e['message_id']));oracle=DelayedLocalReplay(x,P,imu,lever);ref=oracle.base
 for e in ordered:ref=oracle.step(ref,e)
 direct=DelayedLocalReplay(x,P,imu,lever);checkpoint=direct.base;reject=collections.Counter()
 for e in arrived:
  try:checkpoint=direct.step(checkpoint,e)
  except ValueError as err:reject[str(err)]+=1
 lag=DelayedLocalReplay(x,P,imu,lever);reasons=collections.Counter();cost=[]
 for e in arrived:
  tick=time.perf_counter_ns()
  try:reasons[lag.ingest(e)]+=1
  except ValueError as err:reasons[str(err)]+=1
  cost.append((time.perf_counter_ns()-tick)/1e6)
 out=lag.current();delta=difference(out[0],ref[0]);dd=difference(checkpoint[0],ref[0]);return dict(real_events=len(es),duration_s=(int(ts[-1])-int(ts[0]))/1e9,header_order='all qualified sensor events',arrival_order='recorded receipt proxy, callback timestamps unavailable',arrival_rejections=dict(reject),arrival_vs_header_state_delta_norm=float(np.linalg.norm(dd)),arrival_vs_header_P_max_abs=float(np.max(abs(checkpoint[1]-ref[1]))),delay_reasons=dict(reasons),delay_vs_header_state_delta_norm=float(np.linalg.norm(delta)),delay_vs_header_P_max_abs=float(np.max(abs(out[1]-ref[1]))),pending_events=len(lag.events)-len(lag.snapshots),max_buffer_events=lag.max_buffer,reapplied_suffix_operations=lag.replayed,ingest_ms=percentiles(cost),same_common_initialization=True,initialization='first saved local output and preexisting latest IMU; not online startup validation',scope='IMU+wheel sensor-time replay, no map and no ROS executor',online_equivalence=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--local',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(run(a),indent=2)+'\n')
