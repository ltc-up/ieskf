"""Private-input frozen AI comparison. No matching backend or production imports."""
import argparse,json,os,time
from pathlib import Path
import numpy as np
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.objective import state,Objective,violations
from geometry_ieskf.seeded_geometry import SeededUpdate,ndt_seed
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.state import inject,chart_map
from geometry_ieskf.qualification import TrackedCorrespondence,local_optimality,pair_retention,eligible
from geometry_ieskf.partition import margins

def run(root,budget=4,aliases=None):
 root=Path(root);ah=root/'phase_ah_deskew_z_ablation';out=root/'phase_ai_ndt_seeded_geometry';out.mkdir(exist_ok=True)
 selected=json.loads((ah/'selection_private.json').read_text())['selected'];old=json.loads((ah/'frozen_updates_private.json').read_text());ndts=[json.loads(l) for l in (ah/'ndt_results.jsonl').read_text().splitlines()]
 lever=np.array(json.loads((root/'history/phase_v/results/input_contract.json').read_text())['extrinsic']['lever_body_imu_m']);factor=GeometryFactor(lever)
 targetpath=root/'inputs/map_target.bin';target=np.fromfile(targetpath,dtype=np.float32,offset=8).reshape(-1,3).astype(float);corr=TrackedCorrespondence(target);private=[];public=[]
 for e in selected:
  if aliases and e['alias'] not in aliases:continue
  a=e['alias'];record=next(v for v in old if v['alias']==a and v['group']=='A');snap=record['prior'];prior=state(snap);P=np.array(snap['P']);assert prior.t==e['frame']['header_ns'];srcpath=root/'history/phase_v'/e['frame']['source_file'];assert sha(srcpath)==e['frame']['source_sha256'];source=np.load(srcpath)
  n=next(v for v in ndts if v['case_id']==a+'_A');assert n['converged'];seed=ndt_seed(prior,P,np.array(n['final']),lever);common=corr.get(prior,source,factor);fixed=Objective(prior,P,common,factor);U=SeededUpdate(factor)
  def assessment(z):
   c=corr.get(inject(prior,z),source,factor);o=Objective(prior,P,c,factor);r,J,rho,h=o.data(z);m=local_optimality(P,z,J,r,c['w']*h);probe=corr.get(inject(prior,z+m['gn_delta']),source,factor);pair=pair_retention(c,probe);m.update(association_witness=pair['identical'],support_good=bool(c['good']),engineering_safe=not violations(z));cert=eligible(m,C['convergence_whitened_step']);u=np.linalg.solve(o.L,z)
   return dict(own_factor_MAP=o.cost(z),prior_cost=float(.5*u@u),own_geometry=float(c['w']@rho),prediction_factor_MAP=fixed.cost(z),support=c['metrics'],pair_probe=pair,certificate=cert,gn_white_norm=m['gn_white_norm'],newton_decrement=m['newton_decrement'],gradient_white_norm=m['gradient_white_norm'],guard_margins=margins(z),violations=violations(z),delta=z),c
  evaluations={};cs={}
  for label,z in [('prediction',np.zeros(15)),('NDT_seed',seed),('historical_accepted',np.array(record['old_accepted_delta']))]:evaluations[label],cs[label]=assessment(z)
  runs={}
  for label,start in [('prediction',np.zeros(15)),('NDT_seed',seed)]:
   solver=SeededUpdate(factor,{**C,'max_iterations':budget});_,unchanged,info=solver.correct(prior,P,source,corr,start);assert unchanged is P;end=solver.last_delta.copy();ass,cc=assessment(end);runs[label]=dict(info=info,end=ass,trace=solver.trace,endpoint_pairs=np.c_[cc['source_index'],cc['target_ids']].tolist());assert info.get('covariance_updates',0)==0
  ra=runs['prediction'];baseline_equal=ra['info']['reason']==record['info']['reason'];delta_error=float(np.max(abs(ra['end']['delta']-record['old_accepted_delta']))) if record['info']['accepted'] else None
  if budget==4:assert baseline_equal and (delta_error is None or delta_error<1e-8)
  item=dict(alias=a,source_sha256=sha(srcpath),target_sha256=sha(targetpath),prior=snap,source_scalar_rounding_max_m=float(np.max(abs(source-source.astype(np.float32).astype(float)))),ndt=n,seed=seed,evaluations=evaluations,runs=runs,baseline_reason_equal=baseline_equal,baseline_accepted_delta_max_error=delta_error);private.append(item)
  summary=dict(alias=a,source_sha256=sha(srcpath),time_identity_equal=True,ndt_backend_converged=True,ndt_fitness=n['fitness'],seed_pose_safe=not any(x in evaluations['NDT_seed']['violations'] for x in ['translation','rotation']),seed_full_violations=evaluations['NDT_seed']['violations'],source_scalar_rounding_max_m=item['source_scalar_rounding_max_m'],baseline_reason_equal=baseline_equal,baseline_accepted_delta_max_error=delta_error,evaluations={},runs={})
  def strip(v):
   return {k:x for k,x in v.items() if k!='delta'}
  for k,v in evaluations.items():summary['evaluations'][k]=strip(v)
  for k,v in runs.items():summary['runs'][k]=dict(reason=v['info']['reason'],historical_rule_accept=v['info']['accepted'],iterations=v['info']['iterations'],wall_ms=v['info']['wall_ms'],endpoint=strip(v['end']),delta_z_m=float(v['end']['delta'][2]),delta_vz_m_s=float(v['end']['delta'][5]),gyro_bias_norm_rad_s=float(np.linalg.norm(v['end']['delta'][9:12])),accel_bias_norm_m_s2=float(np.linalg.norm(v['end']['delta'][12:15])),state_P_commits=0)
  public.append(summary);print(a,[(k,v['info']['reason'],v['end']['gn_white_norm'],v['end']['certificate']['accepted']) for k,v in runs.items()],flush=True)
  dump(out/f'private_results_{budget}.json',private);dump(out/f'deidentified_results_{budget}.json',public)
 dump(out/'input_identity.json',{str(p.relative_to(root)):sha(p) for p in [ah/'selection_private.json',ah/'frozen_updates_private.json',ah/'ndt_results.jsonl',root/'history/phase_v/results/input_contract.json',targetpath]})
 return public
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p.add_argument('--budget',type=int,choices=[4,16],default=4);a=p.parse_args();assert a.data_root,'Set IESKF_DATA_ROOT or --data-root';run(a.data_root,a.budget,['E1','E5'] if a.budget==16 else None)
