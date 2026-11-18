"""OFFLINE evaluation only; never imported by online runners."""
import argparse,json,sys,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_windows import stats,interpolate,evaluate

def read(path):
 with Path(path).open() as f:
  for l in f:yield json.loads(l)
def longest(mask,t):
 best=cur=0.
 for k,good in enumerate(mask):
  if good:cur=(cur+float(t[k]-t[k-1])*1e-9) if k and mask[k-1] and t[k]-t[k-1]<100_000_000 else .05;best=max(cur,best)
  else:cur=0
 return best

def main(a):
 p=Path(a.root);out=p/'continuous_analysis';out.mkdir(exist_ok=False);be=Path(a.be);data=Path(a.data);ref=np.load(be/'runtime/reference_lap.npz');ev=np.load(data/'phase_as_accuracy_quality/single_lap/evaluation.npz');header=ref['header'];lookup={int(t):i for i,t in enumerate(header)}
 far={k:[] for k in ['ekf','lidar','diagnostic','timing_debug']}
 for row in read(p/'far_recorded_init/records.jsonl'):
  if row['kind'] in far:far[row['kind']].append(row)
 fin={r['header']:r for r in read(p/'far_recorded_init/inputs.jsonl') if r['kind']=='cloud' and not r['pre_initialization']};lid={r['stamp_ns']:r for r in far['lidar']};fdiag={r['stamp_ns']:r['status'] for r in far['diagnostic']}
 sr=list(read(p/'eskf15_recorded_init/states.jsonl'));nr={r['stamp_ns']:r for r in read(p/'eskf15_recorded_init/matches.jsonl') if 'ndt' in r};hist=list(read(be/'results/controlled2_matched_lap/frames.jsonl'));result=dict(windows={},frameworks={},evidence_boundary='0/75/150ms engineering position references, not independently calibrated truth; state interpolation at header times, not proof of per-observation commitment');cases=[]
 sf=far['ekf'];sfwall=np.array([r['received_wall_ns'] for r in sf],dtype=np.int64)
 for name,lo,hi in [('normal8',675,803),('right3',1317,1405)]:
  ids=np.arange(lo,hi+1);grid=header[ids];S={}
  S['FAR_final_state']=interpolate([r['stamp_ns'] for r in sf],[r['position'] for r in sf],Rotation.from_quat([r['quaternion'] for r in sf]).as_matrix(),grid)
  S['new15D_final_state']=interpolate([r['stamp_ns'] for r in sr],[r['position'] for r in sr],Rotation.from_quat([r['quaternion'] for r in sr]).as_matrix(),grid)
  hh=hist[max(0,lo-3):hi+4];S['historical15D_state']=interpolate([r['state_stamp'] for r in hh],[np.array(r['pose'])[:3,3] for r in hh],[np.array(r['pose'])[:3,:3] for r in hh],grid)
  common=np.ones(len(ids),dtype=bool)
  for k,(xyz,R,valid,gap) in S.items():common&=valid&(gap< (200_000_000 if k.startswith('historical') else 100_000_000))
  z=dict(expected_scoring_frames=len(ids),common_grid_valid=int(common.sum()),state={k:evaluate(xyz,R,ids,ref,ev['tangent'],common) for k,(xyz,R,valid,gap) in S.items()},missing_grid={k:ids[~(v[2]&(v[3]<100_000_000))].tolist() for k,v in S.items()},registration={},timing={})
  for label,rows in [('FAR_GICP',lid),('new15D_NDT',nr)]:
   found=np.array([i for i in ids if int(header[i]) in rows],dtype=int);r=[rows[int(header[i])] for i in found]
   if label=='FAR_GICP':xyz=np.array([v['position'] for v in r]);R=Rotation.from_quat([v['quaternion'] for v in r]).as_matrix()
   else:T=np.array([v['ndt']['final'] for v in r]);xyz=T[:,:3,3];R=T[:,:3,:3]
   z['registration'][label]=dict(returned=len(found),missing_indices=[int(i) for i in ids if int(header[i]) not in rows],all_returned_candidates=evaluate(xyz,R,found,ref,ev['tangent']))
   if label=='new15D_NDT':z['registration'][label].update(accepted=sum(bool(v['update'].get('accepted')) for v in r),converged=sum(bool(v['ndt']['converged']) for v in r),invalid_or_uncommitted_indices=[int(i) for i,v in zip(found,r) if not v['update'].get('accepted')])
   else:z['registration'][label]['state_commit_per_id']='UNKNOWN: no per-observation native ledger'
  ft=[];sp=[];dispatch=[];prepares=[]
  for i in ids:
   t=int(header[i])
   if t in lid and t in fin:
    v=lid[t];inp=fin[t];cost=(v['received_wall_ns']-inp['publish_wall_ns'])*1e-6+(inp['publish_sim_ns']-inp['available'])*1e-6;ft.append(cost);idx=np.searchsorted(sfwall,v['received_wall_ns']);sp.append(cost+(sfwall[idx]-v['received_wall_ns'])*1e-6 if idx<len(sfwall) else np.nan);dispatch.append(inp['lateness_ms']);prepares.append(inp['meta']['prepare_ms'])
    if i in [1392,1393]:cases.append(dict(index=int(i),far=ref_case(v,i,ref,ev['tangent']),far_status=fdiag.get(t),far_after_available_ms=cost,raw_ids=[x['id'] for x in inp['identities']]))
  rr=[nr[int(t)] for t in grid if int(t) in nr];timing=dict(FAR_after_available_to_GICP_ms=stats(ft),FAR_first_subsequent_state_proxy_ms=stats(sp),FAR_dispatch_lateness_ms=stats(dispatch),FAR_input_preparation_prefetched_ms=stats(prepares),new15D_after_available_to_commit_ms=stats([v['after_available_ms'] for v in rr]),new15D_NDT_ms=stats([v['ndt_wall_ms'] for v in rr]),new15D_deskew_ms=stats([v['deskew_wall_ms'] for v in rr]),new15D_delayed_update_ms=stats([v['delayed_update_ms'] for v in rr]),over50=dict(FAR_GICP=sum(x>50 for x in ft),FAR_state_proxy=sum(x>50 for x in sp),new15D=sum(v['after_available_ms']>50 for v in rr)))
  z['timing']=timing
  for k,(xyz,R,valid,gap) in S.items():
   err=np.linalg.norm(xyz[:,:2]-ref['G_0.075'][ids,:2],axis=1);z.setdefault('xy_over2m_engineering_flag',{})[k]=dict(frames=int(((err>2)&common).sum()),longest_seconds=longest((err>2)&common,grid))
  result['windows'][name]=z
 for c in cases:
  t=int(header[c['index']]);v=nr.get(t)
  if v:c['new15D_NDT']=dict(pose_error=ref_case(dict(position=np.array(v['ndt']['final'])[:3,3].tolist()),c['index'],ref,ev['tangent']),converged=v['ndt']['converged'],accepted=v['update'].get('accepted'),after_available_ms=v['after_available_ms'])
 for label,rows in [('FAR',sf),('new15D',sr)]:
  times=np.array([r['stamp_ns'] for r in rows]);unique=np.unique(times);result['frameworks'][label]=dict(state_records=len(rows),stamp_gaps_ms=stats(np.diff(unique)/1e6),state_stamp_reversals=int((np.diff(times)<0).sum()),estimated_z_range=[min(r['position'][2] for r in rows),max(r['position'][2] for r in rows)])
 debug=far['timing_debug'];result['FAR_native_components_ms']={k:stats([r['values'][k] for r in debug if r['node']=='LidarOdometry' and k in r['values']]) for k in ['node/cloud_callback_time','pipeline/pipeline_time','registration/registration_time','distortion/undistortion_time','covariance/pose_covariance_time','covariance/twist_covariance_time']};result['FAR_status_counts']=dict(collections.Counter(x['message'] for r in far['diagnostic'] for x in r['status']))
 result['FAR_returned_lidar']=len(lid);result['FAR_published_clouds']=len(fin);result['FAR_missing_output_headers']=[int(t) for t in fin if t not in lid];result['cases']=cases
 result['15D_updates']=dict(collections.Counter(v['update'].get('reason','unknown') for v in nr.values()));result['15D_alignments']=len(nr);result['15D_invalid_NDT']=sum(not v['ndt']['converged'] for v in nr.values())
 result['run_status']={name:json.load(open(p/folder/'run_status.json')) for name,folder in [('FAR','far_recorded_init'),('15D','eskf15_recorded_init')]}
 # Do not republish reconstructive first states in public aggregate metrics.
 private_status=result.pop('run_status');(out/'run_status_copy.json').write_text(json.dumps(private_status,indent=2));result['completion']={name:dict(reason=v['reason'],counts=v['counts'],clock=v['clock'],max_delivery_delay_ms=v.get('max_delivery_delay_ms',v.get('maximum_delivery_delay_ms'))) for name,v in private_status.items()}
 (out/'comparison.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()));print(json.dumps({n:dict(states75={k:q['0.075']['xy']['rmse'] for k,q in v['state'].items()},raw75={k:q['all_returned_candidates']['0.075']['xy']['rmse'] for k,q in v['registration'].items()},timing=v['timing']) for n,v in result['windows'].items()},indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()))
def ref_case(row,i,ref,tangent):
 e=np.array(row['position'])-ref['G_0.075'][i];u=tangent[i];return dict(xy=float(np.linalg.norm(e[:2])),longitudinal=float(u@e[:2]),lateral=float(-u[1]*e[0]+u[0]*e[1]))
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','be','data']];main(p.parse_args())
