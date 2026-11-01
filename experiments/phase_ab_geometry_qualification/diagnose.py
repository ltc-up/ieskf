"""Reconstruct AA frozen iterates, not a continuous replay or parameter search."""
import argparse,json,time,hashlib,collections
from pathlib import Path
import numpy as np
from scipy.linalg import cho_factor,cho_solve,solve_triangular
from scipy.optimize import least_squares
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.state import State,inject,difference,chart_map
from geometry_ieskf.objective import Objective,state,violations
from geometry_ieskf.geometry import GeometryFactor,GeometryCorrespondence
from geometry_ieskf.geometry_factor import weights
from geometry_ieskf.qualification import pair_retention,eligible,atomic_select

class TrackedCorrespondence(GeometryCorrespondence):
 def get(self,x,source,factor):
  c=super().get(x,source,factor);T=factor.transform(x);y=source@T[:3,:3].T+T[:3,3];distance,ids=self.tree.query(y,k=1)
  use=np.flatnonzero(distance<=self.c['correspondence_distance_m']);use=use[self.valid[ids[use]]]
  assert np.array_equal(ids[use],c['target_ids']) and np.array_equal(source[use],c['p'])
  c['source_index']=use;return c

def metrics(o,z):
 r,J,rho,h=o.data(z);w=o.corr['w']*h;A=J@o.L;u=solve_triangular(o.L,z,lower=True);g=u+A.T@(w*r);M=np.eye(15)+A.T@(w[:,None]*A);du=cho_solve(cho_factor(M),-g)
 return {'cost':o.cost(z),'prior_cost':float(.5*u@u),'geometry_cost':float(o.corr['w']@rho),
  'gradient_white_inf':float(np.max(abs(g))),'gradient_white_norm':float(np.linalg.norm(g)),
  'gn_white_norm':float(np.linalg.norm(du)),'newton_decrement':float(np.sqrt(max(0.,-g@du))),
  'gn_delta':o.L@du,'gradient_white':g,'normal_equation_residual':float(np.max(abs(M@du+g))),
  'Lambda_full':J.T@(w[:,None]*J),'information_white_eigenvalues':np.linalg.eigvalsh(A.T@(w[:,None]*A)),
  'support_good':bool(o.corr['good']),'engineering_safe':not violations(z),
  'residual_abs_quantiles':np.percentile(abs(r),[50,90,95,99,100]),'weights':w}

def association(a,b):
 p=pair_retention(a,b);A={int(k):i for i,k in enumerate(a['source_index'])};B={int(k):i for i,k in enumerate(b['source_index'])};common=sorted(set(A)&set(B));dots=[abs(a['n'][A[k]]@b['n'][B[k]]) for k in common]
 p['common_source_fraction']=len(common)/max(1,len(A));p['normal_change_degrees_quantiles']=np.percentile(np.degrees(np.arccos(np.clip(dots,-1,1))),[50,90,100]).tolist() if dots else []
 p['target_set_jaccard']=len(set(a['target_ids'])&set(b['target_ids']))/max(1,len(set(a['target_ids'])|set(b['target_ids'])))
 return p

def posterior(P,z,o,w):
 _,J,_,_=o.data(z);L=o.L;A=J@L;post=L@cho_solve(cho_factor(np.eye(15)+A.T@(w[:,None]*A)),L.T);B=chart_map(z);Q=B@post@B.T;return (Q+Q.T)/2

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();D=Path(a.data_root);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 rawpath=D/'phase_aa_z_validation/new_replay.json';baseline=json.load(open(rawpath));inputs=json.load(open(D/'history/phase_v/results/input_continuous_extension.json'));contract=json.load(open(D/'history/phase_v/results/input_contract.json'));frames={f['header_ns']:f for f in inputs['frames']}
 targetpath=D/'inputs/map_target.bin';assert sha(targetpath)==contract['target_sha256'];query=TrackedCorrespondence(np.fromfile(targetpath,dtype='<f4',offset=8).reshape(-1,3).astype(float));factor=GeometryFactor(contract['extrinsic']['lever_body_imu_m']);tol=C['convergence_whitened_step'];rows=[];cache={};start=time.perf_counter();cost_errors=[];cov_errors=[];guarded=[]
 for j,e in enumerate(baseline['lidar_updates']):
  if not e['accepted'] and j!=1:continue
  frame=frames[e['header_ns']];srcpath=D/'history/phase_v'/frame['source_file'];assert sha(srcpath)==frame['source_sha256'];source=np.load(srcpath);prior=state(e['predicted']);P=np.array(e['predicted']['P']);z=np.array(e['delta']) if e['accepted'] else np.zeros(15)
  reconstructed=[];step_history=[]
  for i,prop in enumerate(e['solver_trace']['proposals']):
   zz=np.array(prop['current_delta']);corr=query.get(inject(prior,zz),source,factor);assert np.array_equal(corr['target_ids'],e['solver_trace']['queries'][i]['target_ids']);o=Objective(prior,P,corr,factor);m=metrics(o,zz);reconstructed.append(corr)
   if i<len(e['iterations']):
    hist=e['iterations'][i];cost_errors.append(abs(m['cost']-hist['cost_before']))
    nextz=np.array(e['solver_trace']['proposals'][i+1]['current_delta']) if i+1<len(e['solver_trace']['proposals']) else z
    if e['accepted'] or i+1<len(e['solver_trace']['proposals']):
     cost_errors.append(abs(o.cost(nextz)-hist['cost_after']));accepted_white=np.linalg.norm(solve_triangular(o.L,nextz-zz,lower=True))
    else:accepted_white=None
    step_history.append({'iteration':i,'fixed_cost_before':hist['cost_before'],'fixed_cost_after':hist['cost_after'],'fixed_decrease':hist['cost_before']-hist['cost_after'],'scale':hist['scale'],'accepted_step_white_norm':accepted_white,'raw_step_white_norm':float(np.linalg.norm(solve_triangular(o.L,np.array(prop['step']),lower=True))),'gradient_white_norm':m['gradient_white_norm'],'newton_decrement':m['newton_decrement']})
  if not e['accepted']:
   # Analyze last attempted state separately; the actual filter still retained its prior.
   z=np.array(e['solver_trace']['proposals'][-1]['current_delta'])
  final=query.get(inject(prior,z),source,factor);o=Objective(prior,P,final,factor);m=metrics(o,z)
  if e['accepted']:
   assert np.array_equal(final['target_ids'],e['solver_trace']['queries'][-1]['target_ids']);post=posterior(P,z,o,m['weights']);cov_errors.append(float(np.max(abs(post-np.array(e['corrected']['P'])))))
  prev=reconstructed[-1];lastassoc=association(prev,final);trialcorr=query.get(inject(prior,z+m['gn_delta']),source,factor);assocw=association(final,trialcorr)
  # Exact witness along the untruncated next direction; no turnover-percentage tuning.
  m['association_witness']=bool(lastassoc['identical'] and assocw['identical']);decision=eligible(m,tol)
  localstep=m['gn_white_norm']<=tol;localdual=m['newton_decrement']<=tol
  row={'ordinal':j,'time_ns':e['header_ns'],'old_accepted':e['accepted'],'old_reason':e['reason'],'source_sha256':frame['source_sha256'],'final_delta':z,'final':m,'last_association':lastassoc,'next_gn_association':assocw,'history':step_history,'raw_step_criterion':bool(localstep),'dual_criterion':bool(localdual),'decision':decision,'sigma_Z_prior':float(np.sqrt(P[2,2])),'sigma_Z_post_old':float(np.sqrt(e['corrected']['P'][2][2])),'last_actual_step_white':step_history[-1]['accepted_step_white_norm'] if step_history else None,'last_two_fixed_decreases':[h['fixed_decrease'] for h in step_history[-2:]],'support':final['metrics'],'reassociated_next_cost':Objective(prior,P,trialcorr,factor).cost(z+m['gn_delta']),'fixed_next_cost':o.cost(z+m['gn_delta'])}
  if e['accepted']:
   cst,cp,atomic_decision=atomic_select(prior,P,state(e['corrected']),np.array(e['corrected']['P']),decision);row['atomic_kept_prior']=not decision['accepted'] and np.array_equal(cp,P) and np.array_equal(cst.v,prior.v) and np.array_equal(cst.p,prior.p)
  rows.append(row);cache[j]=(e,prior,P,z,source,final,o)
 dump(out/'frame_diagnostics.json',rows)
 assert max(cost_errors,default=0)<1e-7 and max(cov_errors,default=0)<1e-9
 accepted=[r for r in rows if r['old_accepted']];partial=[r for r in accepted if r['old_reason']=='MAX_ITERATIONS']
 def dist(vals):return dict(zip(['min','p50','p90','p95','p99','max'],np.percentile(vals,[0,50,90,95,99,100]).tolist()))
 summary={'AA_baseline_sha256':sha(rawpath),'original_accepted':len(accepted),'iteration_limit_count':len(partial),'all_partial_raw_step_pass':sum(r['raw_step_criterion'] for r in partial),'all_partial_dual_pass':sum(r['dual_criterion'] for r in partial),'all_partial_exact_last_pairs':sum(r['last_association']['identical'] for r in partial),'partial_both_local':sum(r['raw_step_criterion'] and r['dual_criterion'] for r in partial),'partial_qualified':sum(r['decision']['accepted'] for r in partial),'all_old_accepted_qualified':sum(r['decision']['accepted'] for r in accepted),'all_old_converged_qualified':sum(r['decision']['accepted'] for r in accepted if r['old_reason']=='CONVERGED'),'reconstruction_max_cost_error':max(cost_errors),'reconstruction_max_P_error':max(cov_errors),'partial_distributions':{k:dist([r['final'][k] for r in partial]) for k in ['gn_white_norm','gradient_white_norm','gradient_white_inf','newton_decrement','cost']},'last_actual_step_white':dist([r['last_actual_step_white'] for r in partial]),'pair_jaccard':dist([r['last_association']['pair_jaccard'] for r in partial]),'post_prior_sigma_Z_ratio':dist([r['sigma_Z_post_old']/r['sigma_Z_prior'] for r in partial]),'frozen_analysis_wall_s':time.perf_counter()-start,'no_continuous_replay':True}
 dump(out/'summary.json',summary);print(json.dumps(summary,indent=2),flush=True)
 # Deterministic diagnostic representatives, never selected by improvement.
 reps=list(dict.fromkeys([0,1,max(accepted,key=lambda r:abs(baseline['lidar_updates'][r['ordinal']]['corrected']['v_imu_map'][2]-baseline['lidar_updates'][r['ordinal']]['predicted']['v_imu_map'][2]))['ordinal'],min(partial,key=lambda r:r['final']['gn_white_norm'])['ordinal'],min(partial,key=lambda r:r['last_association']['pair_jaccard'])['ordinal']]))
 representatives=[]
 for j in reps:
  e,prior,P,z,source,corr,o=cache[j];initial=metrics(o,z);history=[];begin=time.perf_counter()
  def fun(u):
   zz=o.L@u;rr=o.residual(zz);history.append({'cost':float(.5*rr@rr),'z':zz});return rr
  sol=least_squares(fun,solve_triangular(o.L,z,lower=True),jac=lambda u:o.jacobian(o.L@u)@o.L,method='trf',loss='linear',x_scale='jac',ftol=1e-12,xtol=1e-12,gtol=1e-10,max_nfev=300)
  zs=o.L@sol.x;end=metrics(o,zs);dynamic=query.get(inject(prior,zs),source,factor);dyno=Objective(prior,P,dynamic,factor);dyn=metrics(dyno,zs);Ppose=P[np.ix_([0,1,2,6,7,8],[0,1,2,6,7,8])];Lp=np.linalg.cholesky(Ppose);K=np.linalg.solve(Ppose,P[np.ix_([3,4,5,9,10,11,12,13,14],[0,1,2,6,7,8])].T).T;perturbs=[]
  for k in range(6):
   for sign in [-1,1]:
    dp=sign*tol*Lp[:,k];dz=np.zeros(15);dz[[0,1,2,6,7,8]]=dp;dz[[3,4,5,9,10,11,12,13,14]]=K@dp;cc=query.get(inject(prior,z+dz),source,factor);oo=Objective(prior,P,cc,factor)
    perturbs.append({'pose_white_axis':k,'sign':sign,'association':association(corr,cc),'fixed_cost':o.cost(z+dz),'reassociated_cost':oo.cost(z+dz),'reassociated_gradient_white_norm':metrics(oo,z+dz)['gradient_white_norm']})
  representatives.append({'ordinal':j,'old_reason':e['reason'],'initial_fixed':initial,'independent_fixed_solution':end,'fixed_extra_z':zs-z,'fixed_solver_success':bool(sol.success),'fixed_solver_message':sol.message,'fixed_nfev':int(sol.nfev),'fixed_guard_violations':violations(zs),'fixed_objective_history':history,'dynamic_at_fixed_solution':dyn,'association_at_fixed_solution':association(corr,dynamic),'perturbations':perturbs,'wall_ms':1000*(time.perf_counter()-begin),'scope':'fixed-final-association diagnostic; state is not committed'})
 dump(out/'representative_diagnostics.json',representatives)
 # First-frame information accounting at its ACTUALLY committed pose, not an optimized replacement.
 e,prior,P,z,source,corr,o=cache[0];m=metrics(o,z);r,J,rho,h=o.data(z);w=m['weights'];ids=corr['target_ids'];uniq,inv,cnt=np.unique(ids,return_inverse=True,return_counts=True);blocks,bi,bc=np.unique(np.floor(corr['p']/C['source_block_m']).astype(int),axis=0,return_inverse=True,return_counts=True);zinfo=w*J[:,2]**2;blockinfo=np.bincount(bi,weights=zinfo);neighbors=query.tree.query(query.target[uniq],k=C['normal_neighbors'])[1];bw=weights(corr['p'],ids,'block',sigma=C['lidar_sigma_m'])*h;Pactual=posterior(P,z,o,w);Pblock=posterior(P,z,o,bw)
 info={'NUMERIC_INFORMATION_VALID':bool(np.max(abs(Pactual-np.array(e['corrected']['P'])))<1e-9),'GEOMETRY_INFORMATION_TRUSTWORTHY':'NOT_VALIDATED','original_support_points':len(ids),'unique_target_count':len(uniq),'duplicated_target_observations':int(len(ids)-len(uniq)),'max_target_reuse':int(cnt.max()),'target_normalization_error':float(np.max(abs(np.bincount(inv,weights=corr['w'])-1/C['lidar_sigma_m']**2))),'source_5m_block_count':len(blocks),'top_block_fraction_Z_information':float(blockinfo.max()/zinfo.sum()),'top_three_block_fraction_Z_information':float(np.sort(blockinfo)[-3:].sum()/zinfo.sum()),'effective_Z_information_blocks':float(blockinfo.sum()**2/(blockinfo@blockinfo)),'normal_fitting_neighbor_uses':int(neighbors.size),'unique_map_points_in_normal_fits':int(len(np.unique(neighbors))),'overlapping_normal_neighbor_use_fraction':float(1-len(np.unique(neighbors))/neighbors.size),'vertical_normal_fraction_abs_nz_over_08':float(np.mean(abs(corr['n'][:,2])>=.8)),'information_pose6':m['Lambda_full'][np.ix_([0,1,2,6,7,8],[0,1,2,6,7,8])],'prior_P':P,'actual_post_P':Pactual,'block_diagnostic_post_P':Pblock,'sigma_Z_prior':float(np.sqrt(P[2,2])),'sigma_Z_actual':float(np.sqrt(Pactual[2,2])),'sigma_Z_block_diagnostic':float(np.sqrt(Pblock[2,2])),'variance_shrink_factor':float(P[2,2]/Pactual[2,2]),'first_frame_delta':z,'fixed_actual_metrics':m,'first_frame_pair_turnover':association(query.get(prior,source,factor),corr),'block_diagnostic_is_calibrated':False,'no_noise_or_filter_parameter_changed':True}
 np.savez_compressed(out/'first_actual_factors.npz',p=corr['p'],q=corr['q'],n=corr['n'],base_weights=corr['w'],huber_weights=h,H=J,source_index=corr['source_index'],target_ids=ids,z=z,P=P,Pactual=Pactual)
 dump(out/'first_information.json',info);print('Representative fixed-factor diagnostics:',reps,'first sigmaZ/block',info['sigma_Z_actual'],info['sigma_Z_block_diagnostic'],flush=True)
if __name__=='__main__':main()
