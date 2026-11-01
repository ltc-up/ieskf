"""Cycle, feasible-root and posterior-transaction diagnostics; no continuous replay."""
import argparse,json,numpy as np
from pathlib import Path
from scipy.linalg import cho_factor,cho_solve
from compare_states import load_first,association
from geometry_ieskf.common import C,dump
from geometry_ieskf.state import inject,chart_map,difference
from geometry_ieskf.objective import Objective
from geometry_ieskf.capture import assess,starting_seeds
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.qualification import atomic_select,pair_retention
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,f,src,prior,P,factor,oldq=load_first(D);L=np.linalg.cholesky(P)
cycle_summaries=[]
for name,folder,query in [('original',O,oldq),('nearest_valid',O/'nearest_valid',NearestValidCorrespondence(oldq.target))]:
 r=json.load(open(folder/'nominal.json'));hist=r['history'];lags=[]
 for lag in range(1,11):
  errors=[np.linalg.norm(np.linalg.solve(L,np.array(hist[k]['z_after'])-np.array(hist[k-lag]['z_after']))) for k in range(max(lag,len(hist)-15),len(hist))]
  lags.append({'lag':lag,'max_prior_white_repeat_error':float(max(errors))})
 za=np.array(hist[-2]['z_after']);zb=np.array(hist[-1]['z_after']);ca=query.get(inject(prior,za),src,factor);cb=query.get(inject(prior,zb),src,factor);ma={int(k):i for i,k in enumerate(ca['source_index'])};mb={int(k):i for i,k in enumerate(cb['source_index'])};changed=[k for k in ma.keys()&mb.keys() if ca['target_ids'][ma[k]]!=cb['target_ids'][mb[k]]];angles=[np.degrees(np.arccos(np.clip(abs(ca['n'][ma[k]]@cb['n'][mb[k]]),0,1))) for k in changed]
 cycle_summaries.append({'variant':name,'status':r['status'],'final':r['final'],'support':r['final_support'],'lags':lags,'last_edge_pairs':pair_retention(ca,cb),'last_edge_source_entries':len(mb.keys()-ma.keys()),'last_edge_source_exits':len(ma.keys()-mb.keys()),'last_edge_changed_targets':len(changed),'changed_target_normal_degrees_quantiles':np.percentile(angles,[50,90,100]) if angles else [],'final_original_guard_active':r['final_original_guard_KKT']['active']})
dump(O/'cycle_summaries.json',cycle_summaries)
# Exactly predeclared starts, no post-hoc success promotion. Audit the only found interior seed.
results=json.load(open(O/'nearest_valid/seed_results.json'));qualified=[r for r in results if r['interior_qualified']];transaction=[];neighborhood=[]
for r in qualified:
 z=np.array(r['z']);query=NearestValidCorrespondence(oldq.target);corr=query.get(inject(prior,z),src,factor);o,m=assess(prior,P,z,corr,factor);rr,J,rho,h=o.data(z);w=corr['w']*h;A=J@L;Pchart=L@cho_solve(cho_factor(np.eye(15)+A.T@(w[:,None]*A)),L.T);B=chart_map(z);post=B@Pchart@B.T;post=(post+post.T)/2;x,Q,dec=atomic_select(prior,P,inject(prior,z),post,r['certificate']);assert dec['accepted'] and np.linalg.eigvalsh(Q).min()>0;assert np.max(abs(difference(x,prior)-z))<1e-12
 transaction.append({'seed_id':r['seed_id'],'numeric_transaction_pass':True,'sigma_Z_before':float(np.sqrt(P[2,2])),'sigma_Z_conditional_after':float(np.sqrt(Q[2,2])),'minimum_P_eigenvalue':float(np.linalg.eigvalsh(Q).min()),'covariance_updates_hypothetical':1,'actual_filter_updates':0,'state_delta':z,'conditional_P':Q,'label':'UNCONSTRAINED_LOCAL_IRLS_APPROXIMATION_UNCALIBRATED; diagnostic result discarded'})
 for s in starting_seeds(P)[1:]:
  dz=np.array(s['z'])*C['convergence_whitened_step']/max(s['white_radius'],1e-30);cc=query.get(inject(prior,z+dz),src,factor);oo,mm=assess(prior,P,z+dz,cc,factor)
  neighborhood.append({'qualified_seed':r['seed_id'],'direction':s['id'],'association':association(corr,cc),'fixed_cost':o.cost(z+dz),'reassociated_cost':oo.cost(z+dz),'reassociated_GN_white_norm':mm['gn_white_norm']})
cross=[]
for r in qualified:
 for name,query in [('original_rule',oldq),('candidate_rule',NearestValidCorrespondence(oldq.target))]:
  z=np.array(r['z']);c=query.get(inject(prior,z),src,factor);o,m=assess(prior,P,z,c,factor);cross.append({'rule':name,'support_n':len(c['p']),'cost':m['cost'],'gn_white_norm':m['gn_white_norm'],'dual_gradient':m['newton_decrement'],'gradient_white_norm':m['gradient_white_norm']})
dump(O/'qualified_root_cross_rule.json',cross)
dump(O/'conditional_transaction.json',transaction);dump(O/'qualified_neighborhood.json',neighborhood)
# Rejections stay atomic: no covariance is computed or committed for the nominal cycle.
nom=json.load(open(O/'nearest_valid/nominal.json'));x,Q,decision=atomic_select(prior,P,inject(prior,np.array(nom['z'])),P,nom['certificate']);assert not decision['accepted'] and np.array_equal(Q,P) and np.array_equal(x.p,prior.p) and np.array_equal(x.v,prior.v)
# Existing normal predicate reasons at the two original nearest Targets.
boundary=json.load(open(O/'support_boundary_audit.json'));ids=[s['nearest_target_id'] for s in boundary['records'][0]['states']];normal_checks=[]
for tid in ids:
 dd,ii=oldq.tree.query(oldq.target[tid],k=C['normal_neighbors']);pts=oldq.target[ii];dev=pts-pts.mean(0);ev=np.linalg.eigvalsh(dev.T@dev/len(ii));ratio=float(ev[0]/ev.sum());normal_checks.append({'target_id':tid,'max_neighbor_distance_m':float(dd[-1]),'planarity_ratio':ratio,'second_eigenvalue_m2':float(ev[1]),'radius_pass':bool(dd[-1]<=C['normal_radius_m']),'planarity_pass':bool(ratio<=C['normal_planarity_ratio']),'second_eigenvalue_pass':bool(ev[1]>=C['normal_second_eigenvalue_m2'])})
dump(O/'original_target_normal_checks.json',normal_checks)
decision={'continuous_eligible':False,'nominal_interior_stable':nom['interior_qualified'],'reason':'nominal original-prediction run remains periodic/nonstationary; one prespecified perturbed-start success must not replace nominal after seeing outcomes','qualified_first_frame_seeds':len(qualified),'first_frame_seeds_tested':len(results),'posterior_transaction_only_on_diagnostic_interior_seed':len(transaction),'nominal_reject_keeps_state_and_P':True,'new_continuous_runs':0,'longer_replay_eligible':False,'no_posterior_gate_noise_or_guard_changes':True};dump(O/'continuous_decision.json',decision)
print('Qualified first-frame starts',len(qualified),'of',len(results),'neighborhood changes',sum(not n['association']['identical'] for n in neighborhood),'/',len(neighborhood));print(decision);print(normal_checks)
