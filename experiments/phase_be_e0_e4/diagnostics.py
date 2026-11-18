"""Offline diagnostics only: sensor-frame innovations and coupled projection energy."""
import argparse,json,collections
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.optical_velocity import OpticalVelocity
from ndt_global.validation import static_transform
from evaluate import stats
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();c=json.load(open(a.root/'runtime/paths.json'));motion=json.load(open(c['motion']));optical=json.load(open(c['optical']))['events'];os=np.array([e['header_ns'] for e in optical]);imu=[e for e in motion['events'] if e['kind']=='imu'];ts=np.array([e['header_ns'] for e in imu]);T=static_transform(motion['transforms'],'base_link','imu_bosch');lever=T[:3,3];model=OpticalVelocity(lever,static_transform(motion['transforms'],'base_link','kistler'),(0,1));result={}
for name in ['final_matched_lap','final_directional_lap','revised_optical_lap','final_optical_multilap']:
 run=a.root/'results'/name
 if not (run/'summary.json').exists():continue
 rows=[json.loads(s) for s in (run/'frames.jsonl').open()];multi='multilap' in name;fixed=np.load(Path(c['data'])/'phase_as_accuracy_quality'/('multilap' if multi else 'single_lap')/'evaluation.npz');segments=fixed['corner'];groups=collections.defaultdict(lambda:collections.defaultdict(list));retained=collections.Counter()
 for r in rows:
  b=next(v for v in r['branches'] if v['branch']==r['selected']);d=b['diagnostics'];key=str(int(segments[r['index']]));values={}
  if 'ndt' in d:
   L=np.asarray(d['ndt'][1]['information']);v,V=np.linalg.eigh(L);keep=v>max(1e-12,v[-1]*1e-3);values.update(lateral_retained_energy=float(np.sum(V[1,keep]**2)),weak_lateral_alignment=abs(float(V[1,0])),minimum_eigenvalue=float(v[0]),normalized_condition=float(v[-1]/max(v[0],1e-12)));retained[int(keep.sum())]+=1
  if 'slip' in d:values.update(beta_deg=float(np.rad2deg(d['slip']['beta_rad'])),predicted_body_vy=d['slip']['body_velocity'][1],ay_minus_rvx=d['slip']['ay_minus_rvx'])
  oi=int(np.searchsorted(os,r['ref'],side='right')-1);ii=int(np.searchsorted(ts,r['ref'],side='right')-1)
  if oi>=0 and ii>=0 and 0<=r['ref']-os[oi]<=50_000_000 and optical[oi]['status_good']:
   z=b['prior'];x=State(*(np.asarray(z[k]) for k in ['p','v','R','bg','ba']),r['ref']);omega=T[:3,:3]@imu[ii]['omega'];h,_=model.model(x,omega);e=np.asarray(optical[oi]['velocity'])[:2]-h;values.update(raw_optical_x_residual=float(e[0]),raw_optical_y_residual=float(e[1]),optical_age_ms=float((r['ref']-os[oi])*1e-6))
  for g in ['all',key]:
   for k,v in values.items():groups[g][k].append(v)
 result[name]=dict(frames=len(rows),retained_directions=dict(retained),segments={g:{k:stats(v) for k,v in q.items()} for g,q in groups.items()})
(a.root/'results/diagnostics.json').write_text(json.dumps(result,indent=2))
