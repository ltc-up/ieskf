"""Publish only de-identified aggregate diagnostics; private event payloads stay private."""
import sys,json,hashlib
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);public=Path(__file__).resolve().parent
raw=list(map(json.loads,open(root/'results/native_raw_geometry.jsonl')));summ=json.load(open(root/'native_verified_evaluation_summary.json'));tests=json.load(open(root/'results/native_extended_tests.json'));eq=json.load(open(root/'results/native_diagnostic_equivalence.json'));geo=[]
for variant in ['map1013','map0105','map1013_plus_saved']:
 a=[r for r in raw if r['id'].endswith('_'+variant)];quant=lambda v:{k:float(x)for k,x in zip(['min','p50','max'],np.percentile(v,[0,50,100]))};geo.append(dict(variant=variant,n=len(a),support_fraction=quant([r['native_support']/r['source_n']for r in a]),large_given_supported=quant([r['large_white_norm']/max(1,r['native_support'])for r in a]),conditional_lateral_information=quant([np.array(r['XY_Lyaw_fixed_other_projection'])[1,1]for r in a]),nuisance_marginal_lateral_information=quant([np.array(r['XY_Lyaw_schur_nuisance_eliminated'])[1,1]for r in a]),weak_schur_information=quant([np.array(r['schur_eigen']).ravel()[0]for r in a])))
performance=[];Pstatus=[];evidence=[]
for p in sorted((root/'native_verified').glob('*_E[0-4]')):
 rows=list(map(json.loads,open(p/'steps.jsonl')));summary=json.load(open(p.parent/(p.name+'_summary.json')));quant=lambda v:{k:float(x)for k,x in zip(['p50','p95','p99','max'],np.percentile(v,[50,95,99,100]))};perf=dict(id=p.name,wall_ms=summary['wall_ms'],step=quant([r['total_step_ms']for r in rows]),map_read_ms=summary['map_read_ms'],optimization=quant([r['optimization_ms']for r in rows]),marginalization=quant([r['marginalization_ms']for r in rows]),preparation=quant([r['geometry_preparation_ms']for r in rows]),IMU_preintegration=quant([r['imu_preintegration_ms']for r in rows]),full_diagnostic=quant([r['diagnostic_only']['independent_diagnostic_ms']for r in rows]),RSS_MiB=summary['max_rss_KiB']/1024,over50=sum(r['total_step_ms']>50 for r in rows),n=len(rows),peak_active_nodes=max(r['active_nodes']for r in rows),marginalized=rows[-1]['marginalized']);performance.append(perf)
 Ps=[np.array(r['diagnostic_only']['P_X_V_B_explicit'])for r in rows];Pstatus.append(dict(id=p.name,n=len(Ps),min_eigen=float(min(np.linalg.eigvalsh(Q).min()for Q in Ps)),max_asym=float(max(abs(Q-Q.T).max()for Q in Ps)),all_finite=bool(all(np.isfinite(Q).all()for Q in Ps)),final_trace=float(np.trace(Ps[-1]))));evidence.append(dict(id=p.name,LM_outer_counter=quant([r['iterations']for r in rows]),at_or_beyond_config8=sum(r['iterations']>=8 for r in rows),added_imu=sum(r['added_imu']for r in rows),added_wheel=sum(r['added_wheel']for r in rows),added_geometry=sum(r['added_geometry']for r in rows)))
# Keep the synthetic metrics, not numerical synthetic matrices.
test_public={k:v for k,v in tests.items()if not isinstance(v,list)};reference15=json.load(open(root/'reference_15d_predictions_evaluation_summary.json'));reference15=[{k:v for k,v in r.items()if k not in ('timing','update_timing','over50')}|dict(timing='NOT_MEASURED')for r in reference15];Rrows=json.load(open(root/'results/conditional_R_candidates.json'));Rvalid=[r for r in Rrows if'models'in r];Rsummary=dict(total=len(Rrows),valid=len(Rvalid),missing_actual_baseline_R=len(Rrows)-len(Rvalid),calibrated=False,curvature_R_with_cross_terms=sum(np.linalg.norm(np.array(r['models']['conditional_inverse_curvature_NOT_measurement_covariance']['R'])-np.diag(np.diag(r['models']['conditional_inverse_curvature_NOT_measurement_covariance']['R'])))>1e-10 for r in Rvalid))
out=dict(baseline='d418fa2c5c996ec859e310eb2321d8abfc47fe6c',qualification='CPU native raw geometry fixed-lag Shadow only; engineering reference not ground truth',native_commit='8e94654558dfbe7ddb0341de300daaf17e25ee02',tests=test_public,diagnostic_equivalence=eq,geometry=geo,results=summ,existing_15d_prediction_reference=reference15,performance=performance,P_status=Pstatus,optimization=evidence,conditional_R=Rsummary,raw_geometry_checks=dict(n=len(raw),residual_fd_max=max(r['raw_residual_fd_max']for r in raw),H_abs_max=max(r['manual_H_max_diff']for r in raw),H_relative_max=max(r['manual_H_max_diff']/max(1,np.abs(r['native_augmented_H']).max())for r in raw),g_abs_max=max(r['manual_g_max_diff']for r in raw)))
(public/'aggregate_results.json').write_text(json.dumps(out,indent=2,default=lambda z:z.item()if isinstance(z,np.generic)else z));print('published aggregate only',len(summ),'finite configurations')
# Private owner/normal coverage detail: proxy labels, NOT independent sensor evidence.
config=json.load(open(root/'inputs/series.json'));Tbi=np.array(config['T_base_imu']);from scipy.spatial.transform import Rotation
Rbi=Tbi[:3,:3];r=Tbi[:3,3];adj=np.zeros((6,6));adj[:3,:3]=Rbi.T;adj[3:,3:]=Rbi.T;skew=lambda v:np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);adj[3:,:3]=-Rbi.T@skew(r)
plan={r['id']:r for r in json.load(open(root/'inputs/geometric_diagnostic_plan.json'))};owner_summary=[]
for row in raw:
 p=plan[row['id']]
 if not p.get('owner_proxy')or not row['points']:continue
 owners=np.load(p['owner_proxy']);H=[np.zeros((6,6))for _ in range(3)];counts=np.zeros(3,int);miss=np.zeros(3,int);bbox=[[]for _ in range(3)]
 for x in row['points']:
  o=owners[x['i']]
  if not x['associated']:miss[o]+=1;continue
  J=np.array(x['J']);W=np.array(x['W']);H[o]+=adj.T@J.T@W@J@adj;counts[o]+=1
 owner_summary.append(dict(id=row['id'],owner_proxy_only=True,supported=counts.tolist(),missing=miss.tolist(),lateral_information=[m[4,4]for m in H],H_by_owner=[m.tolist()for m in H]))
(root/'results/owner_proxy_information.json').write_text(json.dumps(owner_summary,indent=2,default=lambda z:z.item()if isinstance(z,np.generic)else z))
