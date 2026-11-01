"""Publish only de-identified solver aggregates; no pose trajectory or map payload."""
import argparse,json,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.input);O=Path(a.output);O.mkdir(parents=True,exist_ok=True)
 states=json.load(open(D/'three_states.json'));summary={'phase':'AC','cost_semantics':'Selected-factor Huber MAP sums change factor domains; common-fixed costs are labeled separately. Do not rank accuracy by cost.'}
 summary['three_states']=[{key:s[key] for key in ['name','cost','common_AA_final_factor_cost','gn_white_norm','newton_decrement','gradient_white_norm','delta_z_m','delta_XY_norm_m','delta_map_yaw_rad','engineering_margins','prior_white_radius','association_from_prediction']}|{'support_n':s['support']['support_n'],'source_n':s['support']['source_n']} for s in states]
 variants={};nominals={};P=np.array(json.load(open(D/'three_state_binding.json'))['prior']['P']);L=np.linalg.cholesky(P)
 for name,folder in [('original',D),('nearest_valid',D/'nearest_valid')]:
  rs=json.load(open(folder/'seed_results.json'));reps=json.load(open(folder/'representative_results.json'));nominals[name]=rs[0]
  def safe(r):
   h=r['history'];f=r['final'];return {'start':r.get('seed_id','representative'),'status':r['status'],'interior_qualified':r['interior_qualified'],'outer_iterations':r['outer_iterations'],'gn_white_norm':f['gn_white_norm'],'dual_gradient':f['newton_decrement'],'gradient_white_norm':f['gradient_white_norm'],'selected_factor_cost':f['cost'],'support_n':r['final_support']['support_n'],'initial_support_n':h[0]['support_before']['support_n'] if h else r['final_support']['support_n'],'engineering_margins':f['engineering_margins'],'active_original_guards':r['final_original_guard_KKT']['active'],'original_guard_KKT_status':r['final_original_guard_KKT']['status'],'wall_ms':r['wall_ms']}
  repeats=[]
  expected_lag=2 if name=='original' else 5
  orbit=[np.array(h['z_after']) for h in rs[0]['history'][-expected_lag:]]
  for r in rs:
   if len(r['history'])<20:continue
   hs=r['history'];errs=[float(max(np.linalg.norm(np.linalg.solve(L,np.array(hs[k]['z_after'])-np.array(hs[k-lag]['z_after']))) for k in range(len(hs)-10,len(hs)))) for lag in range(1,11)]
   period=next((i+1 for i,e in enumerate(errs) if e<np.sqrt(np.finfo(float).eps)),None)
   repeats.append({'start':r['seed_id'],'observed_repeat_lag':period,'lag_errors':errs,'distance_to_nominal_recorded_orbit':float(min(np.linalg.norm(np.linalg.solve(L,np.array(r['z'])-v)) for v in orbit))})
  variants[name]={'status_counts':dict(collections.Counter(r['status'] for r in rs)),'seed_results':[safe(r) for r in rs],'representatives':[{'case':label,**safe(r)} for label,r in zip(['rejected_large_proposal','largest_committed_velocity_change','well_converged_control'],reps)],'periodic_endpoint_evidence':repeats,'repeat_evidence_tolerance_semantics':'sqrt(machine epsilon) used only to describe recorded numerical repeats, never as solver acceptance or stopping rule','total_seed_wall_ms':sum(r['wall_ms'] for r in rs),'median_seed_wall_ms':float(np.median([r['wall_ms'] for r in rs]))}
 summary['variants']=variants
 cycle=json.load(open(D/'cycle_audit.json'));boundary=json.load(open(D/'support_boundary_audit.json'));nc=json.load(open(D/'original_target_normal_checks.json'));isolation=json.load(open(D/'selection_order_isolation.json'));cycs=json.load(open(D/'cycle_summaries.json'))
 summary['original_cycle']={k:cycle[k] for k in ['factor_state_matrix','pairs','source_population_difference','max_two_step_repeat_white_distance','min_one_step_white_distance','max_one_step_white_distance','same_state_reassociation_geometry_cost_jump','last10_inner_success','last10_step_scales','last_original_guard_active']}
 summary['original_cycle']['single_focal_point']={'nearest_distances_m':[r['nearest_distance_m'] for r in boundary['records'][0]['states']],'nearest_normal_valid':[r['nearest_target_valid_normal'] for r in boundary['records'][0]['states']],'plane_validity_diagnostics':[{k:x[k] for k in ['max_neighbor_distance_m','planarity_ratio','second_eigenvalue_m2','radius_pass','planarity_pass','second_eigenvalue_pass']} for x in nc],'same_state_ordering_counterfactual':[{name:row[name]['focal_supported'] for name in ['original','nearest_valid']} for row in isolation]}
 summary['cycle_repeats']=[{'variant':x['variant'],'lags':x['lags'],'last_edge_source_entries':x['last_edge_source_entries'],'last_edge_source_exits':x['last_edge_source_exits'],'last_edge_changed_targets':x['last_edge_changed_targets'],'changed_target_normal_degrees_quantiles':x['changed_target_normal_degrees_quantiles'],'last_edge_pairs':x['last_edge_pairs']} for x in cycs]
 tx=json.load(open(D/'conditional_transaction.json'));near=json.load(open(D/'qualified_neighborhood.json'));summary['diagnostic_transactions']=[{k:t[k] for k in ['seed_id','numeric_transaction_pass','sigma_Z_before','sigma_Z_conditional_after','minimum_P_eigenvalue','covariance_updates_hypothetical','actual_filter_updates','label']} for t in tx];summary['qualified_point_neighborhood']={'queries':len(near),'pair_changes':sum(not n['association']['identical'] for n in near),'scope':'12 deterministic pose-white probes at the existing numerical tolerance; no local-perturbation reoptimization or full-neighborhood proof'}
 summary['qualified_root_cross_rule']=json.load(open(D/'qualified_root_cross_rule.json'));ea=json.load(open(D/'candidate_edge_audit.json'));summary['candidate_remaining_boundary']={k:ea[k] for k in ['source_exits','original_radius_m','all_exited_sources_cross_radius_of_previous_valid_target']}
 summary['continuous_decision']=json.load(open(D/'continuous_decision.json'));summary['tests']={'capture_interface':8,'nearest_valid_interface':6,'unique_new_tests_pass':14,'historical_AB_proofs_rerun':0};summary['new_continuous_runs']=0
 (O/'key_results.json').write_text(json.dumps(summary,indent=2))
 # Objective/gradient versus SOLVER iteration is public; no vehicle state path is plotted.
 fig,axes=plt.subplots(2,2,figsize=(11,7))
 for row,(name,r) in enumerate(nominals.items()):
  h=r['history'][-15:];xx=[v['outer']+1 for v in h];axes[row,0].plot(xx,[v['reassociated_selected_factor_cost'] for v in h],'.-',label='Reassociated selected-factor cost');axes[row,0].plot(xx,[v['fixed_cost_after'] for v in h],'.--',label='Current fixed-factor local cost');axes[row,0].set_ylabel(name+' objective');axes[row,0].legend(fontsize=7)
  axes[row,1].semilogy(xx,[v['reassociated_metrics']['gn_white_norm'] for v in h],'.-',label='Untruncated GN');axes[row,1].semilogy(xx,[v['reassociated_metrics']['newton_decrement'] for v in h],'.-',label='Dual gradient');axes[row,1].axhline(1e-4,ls='--',c='r',label='Unchanged tolerance');axes[row,1].legend(fontsize=7)
 for ax in axes.flat:ax.set_xlabel('Outer iteration');ax.grid(alpha=.2)
 fig.suptitle('Different selected factor domains between variants: costs are not accuracy rankings');fig.tight_layout();fig.savefig(O/'nominal_cycles.png',dpi=150);plt.close(fig)
 fig,ax=plt.subplots(figsize=(9,4));labels=['Qualified interior','Budget exhausted','Initial low support'];xx=np.arange(3)
 for off,(name,v) in zip([-.18,.18],variants.items()):
  counts=v['status_counts'];ax.bar(xx+off,[counts.get(s,0) for s in ['INTERIOR_ASSOCIATION_FIXED_POINT','OUTER_BUDGET_EXHAUSTED','LOW_SUPPORT']],width=.36,label=name)
 ax.set_xticks(xx,labels);ax.set_ylabel('Predetermined starts (13 per variant)');ax.legend();fig.tight_layout();fig.savefig(O/'capture_counts.png',dpi=150);plt.close(fig)
 print('Public aggregates written; continuous runs:',summary['new_continuous_runs'])
if __name__=='__main__':main()
