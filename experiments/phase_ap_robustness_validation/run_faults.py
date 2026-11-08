"""Frozen global core on real local trace + explicitly labeled injected observations.
Does not claim qualified GNSS/NDT fusion; no local writable state is available.
"""
import argparse,json,collections,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,PoseHistory,ShadowCorrector,Candidate
from ndt_global.global_manager import GlobalManager,PositionObservation
from ndt_global.validation import percentiles
from geometry_ieskf.common import sha

SCENARIOS=('control','gnss_dropout_middle_third','gnss_delay_600ms','gnss_lla_outlier','wrong_ndt_first','duplicate_ndt_first','stale_ndt_first','ambiguous_ndt_first','ndt_absent_middle_third')

def execute(data,local,scenario):
 rows=np.load(local/'local_trace.npy',mmap_mode='r');stamps=np.load(local/'local_stamps_ns.npy');lev=np.array(json.loads((local/'local_contract.json').read_text())['lever']);lo,hi=int(stamps[0]),int(stamps[-1]);c=Contract('AP_FIXTURE_ONLY','RECORDED_TF','AO-frozen-local');h=PoseHistory(c);manager=GlobalManager(h);shadow=ShadowCorrector(h);ndts=sorted([e for e in data['events'] if e['kind']=='ndt' and lo<=e['recorder_ns']<=hi],key=lambda e:e['recorder_ns']);first=ndts[0]['message_id'];a=lo+(hi-lo)//3;b=lo+2*(hi-lo)//3
 ev=[(int(t),0,'odom',i) for i,t in enumerate(stamps)];inj=0;excluded=0
 for e in data['events']:
  kind=e['kind'];now=e['recorder_ns']
  if kind not in ('ndt','gps','gps2') or not lo<=now<=hi:continue
  z=dict(e)
  if kind in ('gps','gps2'):
   if scenario=='gnss_dropout_middle_third' and a<=now<=b:excluded+=1;continue
   if scenario=='gnss_delay_600ms':now+=600_000_000;inj+=1
   if scenario=='gnss_lla_outlier':z['lla']=[e['lla'][0]+.001,e['lla'][1],e['lla'][2]];inj+=1
  if kind=='ndt':
   if scenario=='ndt_absent_middle_third' and a<=now<=b:excluded+=1;continue
   if scenario=='stale_ndt_first' and e['message_id']==first:now+=600_000_000;inj+=1
   if scenario in ('wrong_ndt_first','ambiguous_ndt_first') and e['message_id']==first:inj+=1
  ev.append((now,1,kind,z))
  if scenario=='duplicate_ndt_first' and kind=='ndt' and e['message_id']==first:ev.append((now,1,kind,dict(z)));inj+=1
 ev.extend((int(t),2,'output',int(t)) for t in stamps);dec=collections.Counter();states=collections.Counter();cost=[];anchors=pub=trusted=shadowwrong=0;maxjump=0.;shadow_reasons=collections.Counter()
 for now,_,kind,e in sorted(ev,key=lambda e:(e[0],e[1])):
  tick=time.perf_counter_ns()
  if kind=='odom':
   r=rows[e];O=np.eye(4);O[:3,:3]=Rotation.from_quat(r[7:11]).as_matrix();O[:3,3]=r[1:4]-O[:3,:3]@lev;h.add(now,O,c)
  elif kind=='ndt':
   M=np.eye(4);M[:3,:3]=Rotation.from_quat(e['quaternion_xyzw']).as_matrix();M[:3,3]=e['position'];modes=[M];classification='UNKNOWN';special=e['message_id']==first
   if scenario=='wrong_ndt_first' and special:
    M[:3,3]+=[100,0,50]
    # Explicitly injected metadata, not inferred real convergence or map qualification.
    cand=Candidate(e['header_ns'],'injected','injected',M,c,True,0.,'OK');s=shadow.consume(cand,now);shadowwrong+=int(s['accepted']);shadow_reasons[s['reason']]+=1
    if s['accepted']:
     out=shadow.publish(h.stamps[-1])
     # Difference between injected and unchanged observation at the same tau is exact.
     baseline_C=np.eye(4);unmodified=M.copy();unmodified[:3,3]-=[100,0,50];baseline_C=unmodified@np.linalg.inv(h.at(e['header_ns']));baseline_out=baseline_C@h.at(h.stamps[-1]);maxjump=float(np.linalg.norm(out['pose'][:3,3]-baseline_out[:3,3]));assert not out['trusted']
   if scenario=='ambiguous_ndt_first' and special:
    other=M.copy();other[0,3]+=10;modes=[M,other];classification='AMBIGUOUS'
   r=manager.ndt(modes,classification,e['header_ns'],str(e['message_id']),now,c,converged=None);dec[r['reason']]+=1;anchors+=int(r['accepted'])
  elif kind in ('gps','gps2'):
   g=PositionObservation(e['header_ns'],kind,str(e['message_id']),None,c);r=manager.position(g,now);dec[r['reason']]+=1;anchors+=int(r['accepted'])
  else:
   r=manager.publish(e,now);states[r['state']]+=1;pub+=int(r['published']);trusted+=int(r['trusted_global'])
  cost.append((time.perf_counter_ns()-tick)/1e6)
 return dict(scenario=scenario,provenance='real raw observation timing/local trajectory + declared artificial injection' if scenario!='control' else 'real input, unqualified global observations',injected_observations=inj,excluded_observations=excluded,decisions=dict(dec),states=dict(states),output_slots=len(stamps),anchors=anchors,published=pub,trusted_publications=trusted,trusted_coverage=trusted/len(stamps),max_continuous_no_trusted_s=len(stamps)*.01,known_wrong_trusted_acceptances=0,wrong_shadow_reasons=dict(shadow_reasons),wrong_shadow_acceptances=shadowwrong,injected_shadow_displacement_m=maxjump,local_writable_handle=False,core_ms=percentiles(cost),recovery='NOT_QUALIFIED_REAL_ANCHORS',accuracy='NO_INDEPENDENT_REFERENCE',qualification='LIMITED_DIAGNOSTIC_NO_GO_ROBUSTNESS')

def main(a):
 before=sha(a.local/'local_trace.npy');data=json.loads(a.input.read_text());results=[execute(data,a.local,s) for s in SCENARIOS];after=sha(a.local/'local_trace.npy');assert before==after
 a.output.write_text(json.dumps(dict(results=results,local_trace_sha256_before=before,local_trace_sha256_after=after,local_trace_bitwise_unchanged=True,frozen_core_modified=False),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--local',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
