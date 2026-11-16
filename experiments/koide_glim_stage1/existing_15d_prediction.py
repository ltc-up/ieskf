"""Read-only conditional short predictions using actual saved15D mean/fullP.
No registration, no pose correction, no write to any retained estimator.
"""
import json,sys
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[2];sys.path.insert(0,str(REPO/'src'))
from geometry_ieskf.state import State
from geometry_ieskf.inertial import InertialPredictor
from ndt_global.delayed_local import DelayedLocalReplay

def require_fresh_anchor(t0,state_time):
 if not 0<=t0-state_time<=20_000_000:raise ValueError('NO_NEARBY_REAL_15D_CHECKPOINT_20MS')

def main(root,existing):
 root=Path(root);old=Path(existing);plans=[json.load(open(root/'plan.json')),json.load(open(root/'be_holdout_qualified/plan.json'))];cases={c['id']:c for p in plans for c in p['cases']};results={}
 for folder in [root,root/'be_holdout_qualified']:
  for r in map(json.loads,open(folder/'results.jsonl')):
   if not r['failures']:results[r['id']]=r
 cfg=json.load(open(old/'phase_bg_far_lio/c5/config.json'));Tbi=np.array(json.load(open(cfg['extrinsics']))['T_base_imu']);lever=Tbi[:3,3];Rb=Tbi[:3,:3];m=json.load(open(plans[0]['IMU_Wheel']));sensor=[e for e in m['events'] if e['kind'] in ('imu','wheel') and e.get('valid')];imus=[e for e in sensor if e['kind']=='imu'];frames={r['ref']:r for r in map(json.loads,open(old/'phase_be_e0_e4/results/controlled2_matched_lap/frames.jsonl'))};queries={}
 for id,c in cases.items():
  if id.startswith('C4') and id in results:
   for b in results[id]['B']:
    if 'coverage'in b:queries[c['t']-int(round(b['dt_ms']*1e6))]=None
 # Only already published C5 snapshots, never a retrospectively smoothed future packet.
 if queries:
  times=sorted(queries)
  for l in open(old/'phase_bg_far_lio/c5/eskf15_1013/output100.jsonl'):
   p=json.loads(l)['packet']
   if not p.get('valid'):continue
   for t in times:
    if p['publish_time_ns']<=t and p['state_time_ns']<=t:queries[t]=p
 out=[]
 def T(x):
  a=np.eye(4);a[:3,:3]=x.R;a[:3,3]=x.p-x.R@lever;return a
 for id,r in results.items():
  c=cases[id]
  for b in r['B']:
   if 'coverage' not in b:continue
   t0=c['t']-int(round(b['dt_ms']*1e6));t1=c['t'];z=dict(id=id,dt_ms=b['dt_ms'],status='UNKNOWN')
   try:
    if id.startswith('BE'):
     pr=frames[t0]['branches'][0]['prior'];x=State(np.array(pr['p']),np.array(pr['v']),np.array(pr['R']),np.array(pr['bg']),np.array(pr['ba']),t0);P=np.array(pr['P']);z['anchor']='actual saved BE scan prior; map-feedback history, availability contract inherited from BE'
    else:
     p=queries.get(t0)
     if p is None:raise ValueError('NO_REAL_15D_CHECKPOINT_FOR_INTERVAL')
     x=State(np.array(p['position_map']),np.array(p['velocity_map']),np.array(p['rotation_map_body']),np.array(p['gyro_bias']),np.array(p['accel_bias']),p['state_time_ns']);P=np.array(p['P']);
     require_fresh_anchor(t0,x.t)
     z['anchor']='actual C5 snapshot published before interval start; C5 q50 self-feedback history, NOT old q11 history'
    im=max((e for e in imus if e['header_ns']<=x.t),key=lambda e:e['header_ns']);held=dict(header_ns=im['header_ns'],omega=Rb@im['omega'],force=Rb@im['force']);replay=DelayedLocalReplay(x,P,held,lever);at0=None
    events=[e for e in sensor if x.t<e['header_ns']<=t1 and e['recorder_ns']<=t1];events.sort(key=lambda e:(e['recorder_ns'],e['message_id']))
    def extrapolate(t):
     y,Q,h=replay.current()
     if y.t<t:y,Q=InertialPredictor().predict(y,Q,h['omega'],h['force'],(t-y.t)*1e-9);y.t=t
     if y.t!=t:raise ValueError('NO_EXACT_CAUSAL_QUERY_STATE')
     return y,Q
    for e in events:
     if at0 is None and e['header_ns']>t0:at0=extrapolate(t0)
     e=e.copy()
     if e['kind']=='imu':e['omega']=Rb@e['omega'];e['force']=Rb@e['force']
     replay.ingest(e)
    if at0 is None:at0=extrapolate(t0)
    end,Q=extrapolate(t1);relative=np.linalg.inv(T(at0[0]))@T(end);z.update(status='SAVED15D_IMU_WHEEL_ONLY_SHORT_PREDICTION',relative=relative.tolist(),P_final=Q.tolist(),P_start=at0[1].tolist(),no_ndt_updates_after_anchor=True,finite=bool(np.isfinite(Q).all()),min_P_eigen=float(np.linalg.eigvalsh(Q).min()))
   except Exception as e:z['reason']=repr(e)
   out.append(z)
 with (root/'existing_15d_short_prediction.json').open('x')as f:json.dump(out,f,indent=2)
 print('real15D short predictions',sum(v['status']!='UNKNOWN'for v in out),'of',len(out))
if __name__=='__main__':main(*sys.argv[1:])
