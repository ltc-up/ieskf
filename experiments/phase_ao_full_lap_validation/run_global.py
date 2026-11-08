"""Whole real stream qualification through unchanged AN manager.
Unknown contracts are not filled with plausible values; history contract is local-only.
"""
import argparse,json,collections,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,PoseHistory
from ndt_global.global_manager import GlobalManager,PositionObservation
from ndt_global.validation import percentiles
from geometry_ieskf.common import sha

def run(a):
 before=sha(a.local/'local_trace.npy');data=json.loads(a.input.read_text());trace=np.load(a.local/'local_trace.npy',mmap_mode='r');stamps=np.load(a.local/'local_stamps_ns.npy');n=len(stamps);contract=Contract('LOCAL_TIMING_FIXTURE_ONLY','RECORDED_STATIC_TF','AO-local');h=PoseHistory(contract);manager=GlobalManager(h);lev=np.array(json.loads((a.local/'local_contract.json').read_text())['lever']);times=[];decisions=collections.Counter();states=collections.Counter();pub=0
 if not n:raise ValueError('NO_LOCAL_OUTPUT')
 # Recorder times are used for global observations. Local availability is idealized
 # Header-time, so this is NOT a real callback timing reconstruction.
 ev=[(int(t),0,'odom',i) for i,t in enumerate(stamps)];ev += [(e['recorder_ns'],1,e['kind'],e) for e in data['events'] if e['kind'] in ('ndt','gps','gps2') and stamps[0]<=e['recorder_ns']<=stamps[-1]];ev += [(int(t),2,'output',int(t)) for t in stamps]
 for now,_,kind,e in sorted(ev,key=lambda x:(x[0],x[1])):
  tick=time.perf_counter_ns()
  if kind=='odom':
   row=trace[e];T=np.eye(4);T[:3,:3]=Rotation.from_quat(row[7:11]).as_matrix();T[:3,3]=row[1:4]-T[:3,:3]@lev;h.add(now,T,contract)
  elif kind=='ndt':
   T=np.eye(4);T[:3,:3]=Rotation.from_quat(e['quaternion_xyzw']).as_matrix();T[:3,3]=e['position'];d=manager.ndt([T],'UNKNOWN',e['header_ns'],str(e['message_id']),now,contract,converged=None);decisions[d['reason']]+=1
  elif kind in ('gps','gps2'):
   g=PositionObservation(e['header_ns'],kind,str(e['message_id']),None,contract);d=manager.position(g,now);decisions[d['reason']]+=1
  else:
   p=manager.publish(e,now);states[p['state']]+=1;pub+=int(p['published'])
  times.append((time.perf_counter_ns()-tick)/1e6)
 after=sha(a.local/'local_trace.npy');assert before==after,'LOCAL_TRACE_MUTATED'
 return dict(local_trace_sha256_before=before,local_trace_sha256_after=after,output_slots=n,output_states=dict(states),observation_reasons=dict(decisions),coordinate_anchors=0,trusted_anchors=0,published=pub,trusted_coverage_fraction=0.,max_continuous_no_trusted_s=(int(stamps[-1])-int(stamps[0])+10_000_000)/1e9,ndt_convergence='UNKNOWN_PUBLIC_TOPIC',gnss_fusion='NOT_QUALIFIED',map_contract='UNVERIFIED',core_ms=percentiles(times),local_trace_sha256=sha(a.local/'local_trace.npy'),local_state_modified=False,online_equivalence=False,accuracy='NOT_VALIDATED',qualification='NO_GO')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--local',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(run(a),indent=2)+'\n')
