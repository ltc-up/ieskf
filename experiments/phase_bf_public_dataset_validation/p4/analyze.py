"""Offline analysis only, after all limited matches are complete."""
import argparse,csv,importlib.util,json
from pathlib import Path
import numpy as np

def mod(name,p):
 s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def yaw(R):return np.arctan2(R[1,0],R[0,0])
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def default(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
def save(p,v):p.write_text(json.dumps(v,default=default,indent=2,allow_nan=False)+'\n')
def main(a):
 root=Path(a.root);out=Path(a.out);out.mkdir(parents=True,exist_ok=False);work=root/'p4';one=work/'ablation_01';two=work/'stability_01';x=json.loads((one/'execution_receipt.json').read_text());y=json.loads((two/'execution_receipt.json').read_text());assert x['calls']==32 and y['calls']==40
 records=x['records']+y['records'];assert len(records)==72
 old=json.loads((one/'historical_full.json').read_text());quality=json.loads((one/'original_source_quality.json').read_text());p2=json.loads((root/'p2/five_01/execution_receipt.json').read_text());priors={r['frame']:r for r in p2['records']}
 for frame in [60,65,76]:
  hist=[r for r in old['P3_OFF'] if r['frame']==frame and r['repeat']>0];base=hist[0];z=dict(base['result'])
  for key in ['align_wall_ms','quality_extra_ms']:z[key]=float(np.median([r['result'][key] for r in hist]))
  records.append(dict(stage='HISTORICAL_FULL',frame=frame,n=z['source_n'],method='full',seed=None,result=z,quality=quality[str(frame)],sampling_order_ms=0.,materialize_ms=0.,source_cache_write_ms=0.,service_ms=None,gpu_process_resident_MiB=None))
 assert len(records)==75
 parent=Path(__file__).parents[1];ad=mod('bf_adapter',parent/'kitti_adapter.py');review=mod('bf_review',parent/'review/analyze_existing.py');ref=np.load(root/'kitti/derived_time_aligned/offline_reference.npz');online=np.load(root/'kitti/derived_time_aligned/online.npz');frames=[60,64,65,66,76]
 G=ad.reference_at(ref['raw_t'],ref['raw_packets'],np.r_[online['scan_t'][0],[priors[f]['request']['header'] for f in frames]])[1:];gt=dict(zip(frames,G));results=[];details=[]
 for r in records:
  f=r['frame'];z=r['result'];T=np.array(z['final']);S=np.array(priors[f]['request']['seed_common']);g=gt[f];u=np.r_[g[:2,0]/np.linalg.norm(g[:2,0]),0];d=np.array([-u[1],u[0],0]);e=T[:3,3]-g[:3,3];e0=S[:3,3]-g[:3,3];delta=T[:3,3]-S[:3,3];H=np.array(z['score_hessian']);geo={L:review.normalize_score(H,np.ravel(z['parameters_internal'])[3:],u,L) for L in [5,10,20]};v=geo[10]['eigenvalues'];q=r['quality'];native=np.array(z['native_internal']);n=r['n']
  row=dict(frame=f,n=n,method=r['method'],seed=r['seed'],stage=r['stage'],iterations=z['iterations'],converged=z['converged'],candidate_valid=z['candidate_valid'],exit=z['exit'],committed=False,match_necessary_conditions=bool(z['converged'] and z['candidate_valid']),fitness_available=False,legacy_R_generated=False,
   seed_long_m=float(u@e0),seed_lateral_m=float(d@e0),seed_yaw_deg=float(np.rad2deg(wrap(yaw(S[:3,:3])-yaw(g[:3,:3])))),seed_xy_m=float(np.linalg.norm(e0[:2])),
   final_long_m=float(u@e),final_lateral_m=float(d@e),final_yaw_deg=float(np.rad2deg(wrap(yaw(T[:3,:3])-yaw(g[:3,:3])))),final_xy_m=float(np.linalg.norm(e[:2])),
   delta_abs_long_m=float(abs(u@e)-abs(u@e0)),delta_abs_lateral_m=float(abs(d@e)-abs(d@e0)),delta_abs_yaw_deg=float(np.rad2deg(abs(wrap(yaw(T[:3,:3])-yaw(g[:3,:3])))-abs(wrap(yaw(S[:3,:3])-yaw(g[:3,:3]))))),delta_xy_m=float(np.linalg.norm(e[:2])-np.linalg.norm(e0[:2])),optimization_long_m=float(u@delta),optimization_lateral_m=float(d@delta),
   score=z['optimizer_score'],valid_pairs=z['valid_pairs'],candidate_pairs=z['candidate_pairs'],pairs_per_source=z['valid_pairs']/n,native_inliers=z['inliers_at_float_pose'],native_supported_fraction=z['inliers_at_float_pose']/n,native_neighbor_points=z['neighbor_points_at_float_pose'],native_residual_scale=z['kappa_at_float_pose'],score_per_source=z['optimizer_score']/n,score_per_pair=z['optimizer_score']/z['valid_pairs'],
   gradient_norm=float(np.linalg.norm(z['score_gradient'])),min_scaled_info=float(v[0]),min_scaled_info_per_source=float(v[0]/n),max_scaled_info=float(v[-1]),scaled_positive_condition=float(v[-1]/v[0]) if v[0]>0 else None,weak_long_alignment=geo[10]['weak_longitudinal_alignment'],native_min_eig=float(np.linalg.eigvalsh(native).min()),native_max_eig=float(np.linalg.eigvalsh(native).max()),
   occupied_voxels_1m=q['occupied_voxels_1m'],occupied_voxels_1_5m=q['occupied_voxels_1_5m'],occupied_strata=q['occupied_strata'],occupied_azimuth=q['occupied_angle_sectors'],near_lt10_fraction=q['near_lt10_fraction'],far_ge40_fraction=q['far_ge40_fraction'],
   align_ms=z['align_wall_ms'],quality_extra_ms=z['quality_extra_ms'],sampling_order_ms=r['sampling_order_ms'],materialize_ms=r['materialize_ms'],source_cache_write_ms=r['source_cache_write_ms'],service_ms=r['service_ms'],gpu_resident_MiB=r['gpu_process_resident_MiB'],timing_origin='historical P3 traceOFF/FitnessOFF steady median2' if r['stage']=='HISTORICAL_FULL' else 'current P3 traceOFF/FitnessOFF single measurement')
  results.append(row);details.append(dict(frame=f,n=n,method=r['method'],seed=r['seed'],quality=q,scaled_geometry=geo,native_internal=native,final_pose=T,seed_pose=S,result=z))
 stability=[]
 for f in frames:
  for n in [3000,5000,10000,20000]:
   rs=[r for r in results if r['frame']==f and r['n']==n and r['method']=='space'];assert len(rs)==3
   stability.append(dict(frame=f,n=n,seeds=[r['seed'] for r in rs],long_min=min(r['final_long_m'] for r in rs),long_max=max(r['final_long_m'] for r in rs),lateral_min=min(r['final_lateral_m'] for r in rs),lateral_max=max(r['final_lateral_m'] for r in rs),yaw_min_deg=min(r['final_yaw_deg'] for r in rs),yaw_max_deg=max(r['final_yaw_deg'] for r in rs),converged_count=sum(r['converged'] for r in rs),max_xy_separation=float(max(np.hypot(r['final_long_m']-s['final_long_m'],r['final_lateral_m']-s['final_lateral_m']) for r in rs for s in rs))))
 save(out/'results.json',dict(baseline='3c1c45e7f3accb7a29008642c192b84d88e2675d',new_calls=72,historical_full_rows=3,evaluated_rows=75,results=results,stability=stability,reference='original OXTS timestamp interpolation, actual scan time, offline after all registrations, no fitted alignment',filter_updates=0,replays=0))
 save(out/'details_private.json',details)
 with (out/'results.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=results[0],lineterminator='\n');w.writeheader();w.writerows(results)
 print('PASS: 72 new matches+3 historical FULL,75 diagnostic rows,20 seed-stability groups; no matches in analysis')
 for f in frames:
  print('FRAME',f)
  for r in results:
   if r['frame']==f and (r['method']!='space' or r['stage']=='A'):print(r['n'],r['method'],round(r['final_long_m'],4),round(r['final_lateral_m'],4),round(r['final_yaw_deg'],4),r['converged'],r['iterations'],r['occupied_voxels_1m'],round(r['far_ge40_fraction'],3),round(r['native_supported_fraction'],3))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--out',required=True);main(p.parse_args())
