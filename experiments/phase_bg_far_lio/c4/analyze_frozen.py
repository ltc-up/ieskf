"""Read saved matches and legacy ledgers only; no estimator or matcher calls."""
import argparse,csv,json
from pathlib import Path
import numpy as np
B=None;L=None

def rows(p):
 with open(p) as f:return list(csv.DictReader(f))
def main(root,archive):
 global B,L
 contract=json.load(open(Path(root)/'input_time_contract.json'));B=int(contract['bag_origin_ns']);L=int(contract['legacy_origin_ns'])
 root=Path(root);a=Path(archive)/'tent_map_hypothesis';hist=rows(a/'closed_loop_1013.csv');lookup={int(x['sensor_stamp_ns']):x for x in hist};out=[]
 for line in open(root/'frozen/results.jsonl'):
  v=json.loads(line);r=v['request'];m=v['result'];p=np.array(m['final']);s=np.array(r['seed_common']);h=lookup.get(r['header']);ref=np.array([float(h['normal_x']),float(h['normal_y'])]) if h else None
  if ref is None and r['group']=='713_original':
   paired=Path(archive)/'fitness_semantics_20261003/runs/critical/gpu0105/captures'/str(r['header'])/'final_pose.txt'
   if paired.exists():ref=np.loadtxt(paired)[:2,3]
  info=-np.array(m['score_hessian']);par=np.array(m['parameters_internal']).ravel();a0,a1=par[3:5];Rx=np.array([[1,0,0],[0,np.cos(a0),-np.sin(a0)],[0,np.sin(a0),np.cos(a0)]]);Ry=np.array([[np.cos(a1),0,np.sin(a1)],[0,1,0],[-np.sin(a1),0,np.cos(a1)]]);J=np.column_stack([np.array([1,0,0]),Rx@np.array([0,1,0]),Rx@Ry@np.array([0,0,1])]);M=np.zeros((6,3));M[:2,:2]=np.eye(2);M[3:,2]=np.linalg.solve(J,np.array([0,0,1]))/10.;I=M.T@info@M;val,vec=np.linalg.eigh((I+I.T)/2)
  out.append(dict(case=r['case_id'],group=r['group'],map=r['identity']['map_variant'],bag_s=(r['header']-B)/1e9,old_s=(r['header']-L)/1e9,header=r['header'],source_n=m['source_n'],converged=m['converged'],exit=m['exit'],iterations=m['iterations'],fitness=m['fitness'],support_ratio=m['inliers_at_float_pose']/m['source_n'],xy_vs_saved_0105=float(np.linalg.norm(p[:2,3]-ref)) if ref is not None else None,seed_xy_vs_saved_0105=float(np.linalg.norm(s[:2,3]-ref)) if ref is not None else None,delta_body=(s[:3,:3].T@(p[:3,3]-s[:3,3])).tolist(),old_final_difference=v['old_final_translation_difference'],align_wall_ms=m['align_wall_ms'],service_wall_ms=v['service_wall_ms'],target_build_ms=m['target_build_ms'],quality_extra_ms=m['quality_extra_ms'],projected_information_eigenvalues_L10=val.tolist(),weak_projected_direction=vec[:,0].tolist(),H_scope='first-order pullback from RxRyRz to world XY/yaw, L=10m; other world directions fixed; score sign reversed, NOT marginalized covariance or exact chart Hessian away from stationary point'))
 (root/'analysis/frozen_scalar_results.json').write_text(json.dumps(out,indent=2))
 # Recover actual legacy first-switch prefix; threshold is descriptive comparison, never online gate.
 q=[x for x in hist if 394<=float(x['t'])<=399];first=next((i for i,x in enumerate(q) if float(x['normal_xy_difference'])>2),None)
 prefix=q[max(0,first-35):min(len(q),first+20)] if first is not None else []
 keys=['sensor_stamp_ns','frame_id','seed_source','seed_x','seed_y','ndt_x','ndt_y','normal_x','normal_y','fitness_score','cov_x','converged','measurement_valid','normal_xy_difference','t']
 small=[{k:x[k] for k in keys}|{'bag_s':(int(x['sensor_stamp_ns'])-B)/1e9,'seed_xy_vs_saved_0105':float(np.hypot(float(x['seed_x'])-float(x['normal_x']),float(x['seed_y'])-float(x['normal_y'])))} for x in prefix]
 (root/'analysis/historical_failure_prefix.json').write_text(json.dumps(small,indent=2))
 oldab=rows(a/'closed_loop_frozen_optimizer_ab.csv');ab=[x for x in oldab if 397<=float(x['t'])<=398 and x['capture_run']=='1013']
 summary=dict(registrations=len(out),groups={},first_legacy_difference_over_2m=small[35] if first is not None and len(small)>35 else None,map_control_legacy=[{k:x[k] for k in ['sensor_stamp_ns','t','map','normal_xy_difference','has_converged','fitness']} for x in ab],time_contract=dict(bag_origin=B,legacy_origin=L,offset_s=(L-B)/1e9,requested_714_936356724_exact_capture_found=False),reference='saved 0105 profile comparison; correlated engineering comparator NOT independent truth')
 for g in sorted(set(x['group'] for x in out)):
  z=[x for x in out if x['group']==g];summary['groups'][g]=dict(n=len(z),converged=sum(x['converged'] for x in z),max_xy_vs_0105=max((x['xy_vs_saved_0105'] for x in z if x['xy_vs_saved_0105'] is not None),default=None),min_support=min(x['support_ratio'] for x in z),max_support=max(x['support_ratio'] for x in z))
 (root/'analysis/frozen_aggregate.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('archive');a=p.parse_args();main(a.root,a.archive)
