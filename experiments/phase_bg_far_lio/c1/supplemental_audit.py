"""Post-run pairing, entire-interval timing and private identity receipts. No estimator."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_windows import stats,interpolate,evaluate

def rows(p):
 with Path(p).open() as f:
  for l in f:yield json.loads(l)
def main(root,be,data):
 p=Path(root);be=Path(be);data=Path(data);out=p/'continuous_analysis';ref=np.load(be/'runtime/reference_lap.npz');fixed=np.load(data/'phase_as_accuracy_quality/single_lap/evaluation.npz');res={};allstates=[];lid={};diag={};debug=[]
 for r in rows(p/'far_recorded_init/records.jsonl'):
  if r['kind']=='ekf':allstates.append(r)
  elif r['kind']=='lidar':lid[r['stamp_ns']]=r
  elif r['kind']=='diagnostic':diag[r['stamp_ns']]=r['status']
  elif r['kind']=='timing_debug':debug.append(r)
 inputs={r['header']:r for r in rows(p/'far_recorded_init/inputs.jsonl') if r['kind']=='cloud' and not r['pre_initialization']};wall=np.array([r['received_wall_ns'] for r in allstates],dtype=np.int64);costs=[];proxies=[];worst=[]
 for t,r in lid.items():
  if t not in inputs:continue
  v=inputs[t];cost=(r['received_wall_ns']-v['publish_wall_ns'])*1e-6+(v['publish_sim_ns']-v['available'])*1e-6;costs.append(cost);k=np.searchsorted(wall,r['received_wall_ns']);proxy=cost+(wall[k]-r['received_wall_ns'])*1e-6 if k<len(wall) else float('nan');proxies.append(proxy);worst.append(dict(header_relative_s=(t-1763205419863639180)/1e9,raw_ids=[x['id'] for x in v['identities']],after_available_ms=cost,first_state_proxy_ms=proxy,status=diag.get(t),dispatch_ms=v['lateness_ms'],prefetched_preparation_ms=v['meta']['prepare_ms']))
 mr=[r for r in rows(p/'eskf15_recorded_init/matches.jsonl') if 'ndt'in r];icpu={k:[] for k in ['imu','wheel','cloud']}
 for r in rows(p/'eskf15_recorded_init/inputs.jsonl'):icpu[r['kind']].append(r['process_ms'])
 res['whole_interval_timing']=dict(FAR_after_available_ms=stats(costs),FAR_first_state_proxy_ms=stats(proxies),FAR_over50=sum(x>50 for x in costs),FAR_state_proxy_over50=sum(x>50 for x in proxies),new15D_after_available_ms=stats([r['after_available_ms'] for r in mr]),new15D_over50=sum(r['after_available_ms']>50 for r in mr),new15D_NDT_ms=stats([r['ndt_wall_ms'] for r in mr]),new15D_deskew_ms=stats([r['deskew_wall_ms'] for r in mr]),new15D_update_ms=stats([r['delayed_update_ms'] for r in mr]),new15D_input_dispatch_process_ms={k:stats(v) for k,v in icpu.items()},FAR_worst_records=sorted(worst,key=lambda x:x['after_available_ms'],reverse=True)[:8])
 res['paired_cold_vs_pit']={}
 for name,lo,hi,folder in [('normal8',675,803,'normal8_retry1'),('right3',1317,1405,'right3')]:
  old=[r for r in rows(p.parent/'results'/folder/'records.jsonl') if r['kind']=='ekf'];ids=np.arange(lo,hi+1);grid=ref['header'][ids];P,R,ok,gap=interpolate([r['stamp_ns'] for r in allstates],[r['position'] for r in allstates],Rotation.from_quat([r['quaternion'] for r in allstates]).as_matrix(),grid);OP,OR,ook,ogap=interpolate([r['stamp_ns'] for r in old],[r['position'] for r in old],Rotation.from_quat([r['quaternion'] for r in old]).as_matrix(),grid);mask=ok&ook&(gap<100_000_000)&(ogap<100_000_000);res['paired_cold_vs_pit'][name]=dict(common_samples=int(mask.sum()),cold=evaluate(OP,OR,ids,ref,fixed['tangent'],mask),pit=evaluate(P,R,ids,ref,fixed['tangent'],mask))
 oldinit=json.load(open(p/'recorded_common_init/initialization.json'));fs=json.load(open(p/'far_recorded_init/run_status.json'))['first_outputs']['ekf'];ps=np.array(fs['pose_cov']).reshape(6,6);vs=np.array(fs['twist_cov']).reshape(6,6)
 res['initialization_checks']=dict(common_mean_source_id=oldinit['source']['message_id'],first_FAR_position_minus_common=np.array(fs['position']).tolist(),first_FAR_position_delta_norm=float(np.linalg.norm(np.array(fs['position'])-oldinit['mean']['position'])),first_FAR_pose_covariance_diagonal=ps.diagonal().tolist(),first_FAR_linear_velocity_covariance_diagonal=vs.diagonal()[:3].tolist(),first_FAR_state_velocity=fs['velocity'],pose_block_symmetric=bool(np.allclose(ps,ps.T)),linear_velocity_block_symmetric=bool(np.allclose(vs[:3,:3],vs[:3,:3].T)),native_initial_internal_P='zero9x9 from audited constructor; no R/Q setter overwrites P',full_native9x9_cross_covariance_exported=False,first_FAR_stamp_minus_init_ms=(fs['stamp_ns']-oldinit['init_ns'])/1e6)
 # Preserve complete initialization receipt privately; public aggregate has no trajectory mean.
 res['initialization_checks'].pop('first_FAR_position_minus_common')
 res['backend_components_us']={k:stats([r['values'][k] for r in debug if r['node']=='StateEstimation' and k in r['values']]) for k in ['runtime/step','runtime/delay_compensation','runtime/publish']}
 res['FAR_debug_geometry']={k:stats([r['values'][k] for r in debug if r['node']=='LidarOdometry' and k in r['values']]) for k in ['registration/num_iter','registration/num_points_frame','registration/converged','distortion/valid','map/num_points']}
 (out/'supplemental.json').write_text(json.dumps(res,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()))
 print('whole interval',res['whole_interval_timing']);print('paired',[(n,x['cold']['0.075']['xy']['rmse'],x['pit']['0.075']['xy']['rmse']) for n,x in res['paired_cold_vs_pit'].items()])
if __name__=='__main__':main(*sys.argv[1:])
