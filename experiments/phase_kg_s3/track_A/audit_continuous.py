"""Independent streaming audit of complete coordinator A replay artifacts.

No reference or saved posterior enters any online algorithm. This helper checks
actual retained outputs, labels numerical health separately from correctness,
and never drops rejected/error frames from cycle denominators.
"""
from log_io import open_text
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np

def stats(values):
 if not values:return None
 a=np.asarray(values,float)
 return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()),min=float(a.min()))

def audit(root):
 root=Path(root);cfg=json.loads((root/'config.frozen.json').read_text())
 reasons=Counter();invalid=[];anomalies=[];cycles=[];deltas=[];fit=[];Rmin=[];Rmax=[];postPmin=[];postPmax=[];cross=[];blocks=[]
 zero_prior_nonzero_geom=0;zero_both_gradients=0;selections=Counter();geom_prior_ratios=[];gradient_angles=[];geometric_contributions=[];prior_contributions=[];selected_scores=[];selected_pairs=[]
 frames=0;accepted=0;uncal=0;source_mismatches=0;wrong_updates=0;all_floor=0;floor_dimensions=0;sandwich_dimensions=0;raw_variances=[]
 for line in open_text(root/'frames.jsonl'):
  f=json.loads(line)
  if f.get('no_alignment'):continue
  frames+=1;u=f['update'];N=f['ndt'];yes=u.get('accepted',False);accepted+=yes
  reasons[u.get('reason','MISSING_REASON')]+=1
  cycles.append(f['full_cycle_ms'])
  if N.get('fitness') is not None:fit.append(N['fitness'])
  if yes:
   uncal+=u.get('covariance_calibrated') is False
   source_mismatches+=u.get('source_version')!=f['source_sha256']
   wrong_updates+=u.get('covariance_updates')!=1
   if cfg.get('strategy',cfg.get('mode'))=='A1' and not N.get('candidate_valid',True):invalid.append(dict(ref_ns=f['ref_ns'],exit=N.get('exit'),fitness=N.get('fitness')))
   if 'selected' in u:selections[u['selected']]+=1
   m=u.get('selected_metrics',{})
   for key,dest in [('score',selected_scores),('valid_pairs',selected_pairs),('prior_contribution',prior_contributions),('geometry_contribution',geometric_contributions),('gradient_angle_rad',gradient_angles)]:
    if m.get(key) is not None:dest.append(m[key])
   if m.get('prior_gradient_norm',0)>0:geom_prior_ratios.append(m['geometry_gradient_norm']/m['prior_gradient_norm'])
   elif 'prior_gradient_norm' in m:
    zero_prior_nonzero_geom+=m['geometry_gradient_norm']>0
    zero_both_gradients+=m['geometry_gradient_norm']==0
   if 'delta' in u:deltas.append(float(np.linalg.norm(np.asarray(u['delta'])[:3])))
   if 'spatial_blocks' in u:blocks.append(u['spatial_blocks'])
   if 'sandwich_variance_eigenvalues' in u:
    raw=np.asarray(u['sandwich_variance_eigenvalues']);raw_variances.extend(raw.tolist())
    floor_dimensions+=int((raw<=1).sum());sandwich_dimensions+=len(raw);all_floor+=bool((raw<=1).all())
   if 'R' in u:
    ev=np.linalg.eigvalsh(np.asarray(u['R']));Rmin.append(float(ev.min()));Rmax.append(float(ev.max()))
  Q=np.asarray(f['current']['P']);error=None
  if Q.shape!=(15,15) or not np.isfinite(Q).all():error='INVALID_FULL_P'
  else:
   ev=np.linalg.eigvalsh((Q+Q.T)/2);postPmin.append(float(ev.min()));postPmax.append(float(ev.max()))
   cross.append(float(np.linalg.norm(Q[:3,3:])))
   if ev.min()<-1e-9:error='INDEFINITE_FULL_P'
   if abs(Q-Q.T).max()>1e-9:error='ASYMMETRIC_FULL_P'
  if error:anomalies.append(dict(ref_ns=f['ref_ns'],reason=error))
 result=dict(update_record_semantics='ingestion-time; later historical relinearization requires lifecycle evidence',zero_prior_nonzero_geometry_gradient_frames=zero_prior_nonzero_geom,zero_both_gradient_frames=zero_both_gradients,selected_candidate_counts=dict(selections),selected_score=stats(selected_scores),selected_valid_pairs=stats(selected_pairs),prior_contribution=stats(prior_contributions),geometry_contribution=stats(geometric_contributions),geometry_to_nonzero_prior_gradient_norm_ratio=stats(geom_prior_ratios),gradient_angle_rad=stats(gradient_angles),sandwich_all_directions_at_conditional_variance_floor_frames=all_floor,sandwich_floor_dimension_fraction=floor_dimensions/sandwich_dimensions if sandwich_dimensions else None,raw_normalized_sandwich_variance=stats(raw_variances),frames=frames,accepted=accepted,reason_counts=dict(reasons),all_accepted_explicitly_uncalibrated=uncal==accepted,accepted_covariance_update_count_anomalies=wrong_updates,source_version_mismatches=source_mismatches,accepted_original_candidate_invalid=invalid,fullP_anomalies=anomalies,complete_cycle_ms=stats(cycles),over_50ms=sum(x>50 for x in cycles),fitness_diagnostic_only=stats(fit),accepted_position_delta_norm_m=stats(deltas),spatial_blocks=stats(blocks),posterior_P_eigen_min=stats(postPmin),posterior_P_eigen_max=stats(postPmax),posterior_position_cross_norm=stats(cross),projected_R_eigen_min=stats(Rmin),projected_R_eigen_max=stats(Rmax),covariance_statistically_calibrated=False,correct_basin_certified_by_numerics=False,real_time_publication_certified=False)
 status=root/'run_status.json'
 if status.exists():result['run_status']=json.loads(status.read_text())
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('--out');a=p.parse_args();text=json.dumps(audit(a.root),indent=2)
 if a.out:Path(a.out).write_text(text+'\n')
 print(text)
