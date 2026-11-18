"""Offline-only engineering reference. Never import this module in a filter."""
import argparse,json,csv,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State,inject
from ndt_global.validation import static_transform

def stats(v):
 v=np.asarray(v,float);v=v[np.isfinite(v)]
 return dict(n=len(v),rmse=float(np.sqrt(np.mean(v*v))),p50=float(np.percentile(abs(v),50)),p95=float(np.percentile(abs(v),95)),p99=float(np.percentile(abs(v),99)),max=float(np.max(abs(v)))) if len(v) else dict(n=0)
def native(b,lever,stamp):
 p=b['prior'];x=State(*(np.asarray(p[k]) for k in ['p','v','R','bg','ba']),stamp)
 if b['update']['accepted']:x=inject(x,np.asarray(b['update']['delta']))
 return x.p-x.R@lever,x.R

def evaluate(root,run):
 paths=json.load(open(root/'runtime/paths.json'));data=json.load(open(paths['motion']));lever=static_transform(data['transforms'],'base_link','imu_bosch')[:3,3]
 rows=[json.loads(s) for s in (run/'frames.jsonl').open()];n=len(rows);multi='multilap' in run.name;reference=np.load(root/'runtime'/('reference_multilap.npz' if multi else 'reference_lap.npz'));fixed=np.load(Path(paths['data'])/'phase_as_accuracy_quality'/('multilap' if multi else 'single_lap')/'evaluation.npz');ns=np.array([v['ref'] for v in rows]);assert np.array_equal(ns,reference['header'][:n]);tan=fixed['tangent'][:n];corner=fixed['corner'][:n]
 posts=[];Rs=[];selected=[]
 for row in rows:
  b=next(b for b in row['branches'] if b['branch']==row['selected']);q,R=native(b,lever,row['ref']);posts.append(q);Rs.append(R);selected.append(b)
 posts=np.asarray(posts);Rs=np.asarray(Rs);out=dict(yaw_reference_contract='common saved attitude at scan time; position-only GNSS lag sensitivity, attitude lag not calibrated',frames=n,complete=(run/'summary.json').exists(),reference='GNSS/attitude engineering proxy after antenna lever conversion; not independent certified truth',sensitivity={})
 csvrows=[]
 for delay in ['0.0','0.075','0.15']:
  G=reference['G_'+delay][:n];ok=reference['raw_valid_'+delay][:n];Rref=reference['R'][:n];e=posts[:,:2]-G[:,:2];lat=-tan[:,1]*e[:,0]+tan[:,0]*e[:,1];lon=np.sum(tan*e,axis=1);xy=np.linalg.norm(e,axis=1);yaw=Rotation.from_matrix(Rs).as_euler('xyz')[:,2];ry=Rotation.from_matrix(Rref).as_euler('xyz')[:,2];dy=np.arctan2(np.sin(yaw-ry),np.cos(yaw-ry))
  out['sensitivity'][delay]=dict(valid=int(ok.sum()),xy=stats(xy[ok]),lateral=stats(lat[ok]),longitudinal=stats(lon[ok]),yaw_deg=stats(np.rad2deg(dy[ok])),segments={str(k):dict(xy=stats(xy[ok&(corner==k)]),lateral=stats(lat[ok&(corner==k)]),longitudinal=stats(lon[ok&(corner==k)])) for k in range(10)})
  if delay=='0.075':
   ndt=np.array([b['ndt']['final'] for b in selected]);ne=np.linalg.norm(ndt[:,:2,3]-G[:,:2],axis=1);good=ok&(ne<=.5);bad=ok&(ne>=2.);accepted=np.array([b['update']['accepted'] for b in selected]);out['proxy_admission']=dict(label='proxy thresholds 0.5m/2m frozen from BD, NOT independent correctness',proxy_good=int(good.sum()),proxy_bad=int(bad.sum()),ambiguous=int((ok&~good&~bad).sum()),missing=int((~ok).sum()),good_accepted=int((good&accepted).sum()),good_rejected=int((good&~accepted).sum()),bad_accepted=int((bad&accepted).sum()),bad_rejected=int((bad&~accepted).sum()))
   for i,row in enumerate(rows):csvrows.append([i,int(ns[i]),int(corner[i]),row['selected'],bool(ok[i]),*posts[i],float(xy[i]),float(lat[i]),float(lon[i]),float(np.rad2deg(dy[i])),bool(accepted[i])])
 # Separate the delivered current state from the scan-time reconstructed posterior.
 stamps=np.array([r['state_stamp'] for r in rows]);headers=reference['header'];right=np.clip(np.searchsorted(headers,stamps),1,len(headers)-1);left=right-1;alpha=(stamps-headers[left])/(headers[right]-headers[left]);inside=(stamps>=headers[0])&(stamps<=headers[-1]);published=np.asarray([r['pose'] for r in rows]);out['published_current']={'reference_contract':'linear interpolation of offline engineering reference at actual state stamp; no extrapolation','outside_reference_span':int((~inside).sum()),'sensitivity':{}}
 for delay in ['0.0','0.075','0.15']:
  G=reference['G_'+delay];ok=inside&reference['raw_valid_'+delay][left]&reference['raw_valid_'+delay][right];target=G[left]*(1-alpha[:,None])+G[right]*alpha[:,None];e=published[:,:2,3]-target[:,:2];t=fixed['tangent'][left]*(1-alpha[:,None])+fixed['tangent'][right]*alpha[:,None];t/=np.linalg.norm(t,axis=1)[:,None];lat=-t[:,1]*e[:,0]+t[:,0]*e[:,1];lon=np.sum(t*e,axis=1);out['published_current']['sensitivity'][delay]=dict(valid=int(ok.sum()),xy=stats(np.linalg.norm(e[ok],axis=1)),lateral=stats(lat[ok]),longitudinal=stats(lon[ok]))
 times=[json.loads(s)['cycle_ms'] for s in (run/'inclusive_timing.jsonl').open()];out['cycle_ms']=stats(times);out['over_50ms']=int(sum(t>50 for t in times));out['components_ms']={k:stats([r['times'].get(k,0.) for r in rows]) for k in sorted({k for r in rows for k in r['times']})}
 accounted=['deskew_ms','matcher_wall_ms','diagnostic_validation_ms','delayed_update_ms','gmf_ms','rollback_replay_ms'];out['components_ms']['other_cpu_io_derived_ms']=stats([max(0.,t-sum(r['times'].get(k,0.) for k in accounted)) for t,r in zip(times,rows)]);out['other_cpu_io_contract']='derived residual: input propagation, copying, source preparation and logging; service overlap excluded from subtraction'
 out['switches']=sum(a['selected']!=b['selected'] for a,b in zip(rows,rows[1:]));out['branch_counts']=dict(collections.Counter(len([b for b in r['branches'] if b['branch']!='shadow_gicp']) for r in rows));out['max_weight']=stats([max(r['weights']) for r in rows]);out['replay_validation']='full mean/P validated in runtime; synthetic exact rollback tests separate'
 out['native_pose_step_m']=stats(np.linalg.norm(np.diff(posts,axis=0),axis=1));out['odom_step_m']=stats([np.linalg.norm(np.asarray(b['odom'])[:3,3]-np.asarray(a['odom'])[:3,3]) for a,b in zip(rows,rows[1:])]);out['max_returned_observation_delay_ms']=max((r['complete']-r['ref'])*1e-6 for r in rows)
 (run/'metrics.json').write_text(json.dumps(out,indent=2))
 with (run/'trajectory_and_errors.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['index','stamp_ns','segment','selected','reference_valid','x','y','z','xy_proxy_error_m','lateral_proxy_error_m','longitudinal_proxy_error_m','yaw_proxy_error_deg','accepted']);w.writerows(csvrows)
 np.savez_compressed(run/'native_trajectory.npz',header=ns,position=posts,rotation=Rs)
 return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();z=evaluate(a.root,a.run);print(a.run.name,z['frames'],z['sensitivity']['0.075']['xy'],z['cycle_ms'])
