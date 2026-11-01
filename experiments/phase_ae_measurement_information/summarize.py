"""Explicit deidentified publication whitelist. Never export private states/matrices/IDs."""
import argparse,json,shutil
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--private-root',required=True);p.add_argument('--public-root',required=True);a=p.parse_args();D=Path(a.private_root);E=Path(a.public_root);load=lambda p:json.load(open(D/p));baseline={k:load('baseline_'+k+'.json') for k in ['G1','G2']};ad=json.load(open(D.parent/'phase_ad_association_objective/precision_results.json'));ad_by_id={r['seed_id']:r for r in ad};binding=json.load(open(D.parent/'phase_ac_geometry_capture/three_state_binding.json'));P_frozen=np.array(binding['prior']['P']);L_frozen=np.linalg.cholesky(P_frozen)
def metric(r):
 d=r['decomposition'];c=r['curvature'];m=d['metrics'];out={'cost':r['cost'],'gradient_white_inf':r['gradient_white_inf'],'support_n':m['support_n'],'source_n':m['source_n'],'candidate_pairs':m['candidate_pairs'],'unique_candidate_target_RECORD_ids':m['unique_targets'],'mixture_plane_mass':m['mixture_plane_mass'],'SPD':c['SPD'],'eigenvalues_white':c['eigenvalues'],'condition_white':c['condition_absolute'],'curvature_scope':c['sigma_status'],'force_white_norms':{k:float(np.linalg.norm(v)) for k,v in d['forces_white'].items()},'total_gradient_white_norm':float(np.linalg.norm(d['total_gradient_white'])),'reconstruction_relative_at_1e_5':d['reconstruction_relative'],'signed_part_frobenius_white':{k:float(np.linalg.norm(M)) for k,M in d['parts_white'].items()}}
 if c['SPD']:out.update(sigma_xyz_curvature_m=c['sigma_xyz'],sigma_rpy_curvature_deg=np.degrees(c['sigma_rpy']).tolist(),K_vz_Z=c['K_vz_Z'],P_vz_Z=c['P_vz_Z'],rho_Z_vz=c['rho_Z_vz'])
 return out

def relative(r,ref):
 dp=np.array(r['p_map'])-np.array(ref['p_map']);dr=(np.array(r['RPY_map'])-np.array(ref['RPY_map'])+np.pi)%(2*np.pi)-np.pi
 return {'translation_difference_norm_m':float(np.linalg.norm(dp)),'absolute_XYZ_difference_m':abs(dp).tolist(),'absolute_RPY_difference_deg':abs(np.degrees(dr)).tolist()}
fixed=[]
for arm in ['A','B','C','D']:
 for root in ['G1','G2']:
  r=load('fixed/'+arm+'_'+root+'_frozen.json');fixed.append({'arm':arm,'root':root,**metric(r)})
nominal=[];arms=[]
for arm in ['B','C','D']:
 records=[];cluster=[]
 for seed in ad:
  r=load('fixed/'+arm+'_'+seed['seed_id']+'.json');f=r['final'];z=np.array(f['z']);base=baseline['G1' if np.linalg.norm(np.linalg.solve(L_frozen,np.array(seed['z'])-np.array(ad[0]['z'])))<1e-4 else 'G2']
  # Compare against same original-AD endpoint, not an arbitrary favorable root.
  shift=relative(f,base);gid=None
  if r['status']=='INTERIOR_STATIONARY_PROBE':
   L=np.linalg.cholesky(np.array(json.load(open(D.parent/'phase_ac_geometry_capture/three_state_binding.json'))['prior']['P']))
   for j,center in enumerate(cluster):
    if np.linalg.norm(np.linalg.solve(L,z-center['z']))<=1e-4:gid=j;center['n']+=1;break
   if gid is None:gid=len(cluster);cluster.append({'z':z,'n':1,'cost':f['cost']})
  records.append({'seed_id':seed['seed_id'],'status':r['status'],'raw_status':r['raw']['status'],'raw_stages':r['raw']['stages'],'same_seed_AD_solution_difference':shift,'cluster':gid,**metric(f)})
  if seed['seed_id']=='nominal':nominal.append({'layer':'fixed_planes','arm':arm,'relative_to_original_G1':relative(f,baseline['G1']),**metric(f)})
 arms.append({'arm':arm,'statuses':dict(Counter(r['status'] for r in records)),'raw_statuses':dict(Counter(r['raw_status'] for r in records)),'clusters':[{'ordinal':j,'n':c['n'],'cost':c['cost']} for j,c in enumerate(cluster)],'all13':records})
refit=[]
for arm in ['B','C','D']:
 rr=load('refit/'+arm+'_nominal.json');f=rr['final'];fixed_nom=load('fixed/'+arm+'_nominal.json')['final']
 refit.append({'arm':arm,'status':rr['status'],'raw_status':rr['raw']['status'],'relative_to_original_G1':relative(f,baseline['G1']),'normal_refit_extra_relative_to_same_density_fixed_planes':relative(f,fixed_nom),'metrics':metric(f),'normal_changes_at_same_G1_G2':load('refit/'+arm+'_normal_changes.json'),'frozen_root_evaluations':[{'root':root,**metric(load('refit/'+arm+'_'+root+'_frozen.json'))} for root in ['G1','G2']]})
dirs=[]
for r in load('directional_information.json'):
 dirs.append({'root':r['root'],'physical_directions':[{k:v for k,v in d.items() if k!='white_profile_direction'} for d in r['directions']],'pose_white_eigenvalues':r['pose_white_eigenvalues'],'standardized_axis_energy_loadings':r['standardized_axis_loadings'],'hessian_stencil_sweep':r['hessian_stencil_sweep'],'nearest_Huber_breakpoint_residual_gap_m':r['nearest_Huber_breakpoint_residual_gap_m']})
sy=[]
for r in load('synthetic/synthetic_results.json'):
 t=r['truth'];f=r['fit']['final'];sy.append({'arm':r['arm'],'target_records':r['target_records'],'source_n':t['decomposition']['metrics']['source_n'],'truth_energy':t['cost'],'truth_white_gradient_XY':t['decomposition']['total_gradient_white'][:2],'truth':metric(t),'optimized':metric(f),'optimized_XY_drift_from_known_truth_m':float(np.linalg.norm(np.array(f['delta_XYZ'])[:2])),'plane_intrinsic_white_H_XX_YY_YawYaw':[np.array(t['decomposition']['parts_white']['plane_intrinsic'])[i,i] for i in [0,1,8]],'total_white_H_XX_YY_YawYaw':[np.array(t['decomposition']['H_white'])[i,i] for i in [0,1,8]]})
vr=load('validation/validation_results.json');completed=load('validation/completed_validation.json')
def score_strip(s):return {k:v for k,v in s.items() if not k.startswith('private_')}
validation={'source_train_n':vr['source_train_n'],'source_validation_n':vr['source_validation_n'],'map_train_n':vr['map_train_n'],'map_validation_n':vr['map_validation_n'],'map_center_min_gap_m':vr['map_center_min_gap_m'],'normal_neighbor_overlap_n':vr['normal_neighbor_overlap_n'],'train_valid_planes_n':vr['train_valid_planes_n'],'validation_valid_planes_n':vr['validation_valid_planes_n'],'independent_absolute_reference':False,'historic_root_scores_contaminated':True,'fixed_witness_selection':completed['fixed_witness_selection'],'all6_train_pairs':completed['all6_train_pairs'],'all_completed_scores':[{k:v for k,v in r.items() if not k.startswith('private_')} for r in completed['rows']],'train_only_all_predeclared_blocks':[{'arm':r['arm'],'train':score_strip(r['train_score']),'validation':score_strip(r['validation_score'])} for r in vr['train_fits']]}
H1=np.array(baseline['G1']['decomposition']['H_white']);H2=np.array(baseline['G2']['decomposition']['H_white']);logmass=-(baseline['G2']['cost']-baseline['G1']['cost'])-.5*(np.linalg.slogdet(H2)[1]-np.linalg.slogdet(H1)[1])
definition=load('density_definition_private.json');repro=load('AD_REPRODUCIBILITY.json')
result={'phase':'AE','publication_scope':'independent code, synthetic controls and deidentified relative/statistical aggregates; no original map/point/state/trajectory/device payload','AD_reproducibility':repro,'density_definition':{'original_records':definition['original_n'],'candidate_pool_n':definition['candidate_pool_n'],'selected_dense_cell_count':len(definition['selected_dense_cells']),'records_in_selected_dense_cells':definition['dense_original_n'],'record_counts':repro['arms_counts'],'original_records_removed_in_D':repro['arms_counts']['A']-repro['arms_counts']['D'],'sampling_scope':'fixed normals first; separately refit original20-neighbor rules'},'frozen_root_density_evaluations':fixed,'density_optimization':arms,'nominal_fixed_plane_sensitivity':nominal,'normal_refit':refit,'directional_Hessian_information':dirs,'synthetic_known_plane':sy,'validation':validation,'duplicate_algebra_checks':load('multiplicity_algebra.json'),'local_mode_mass_note':{'AD_energy_local_Laplace_log_mass_G2_over_G1':float(logmass),'scope':'hypothetical literal energy in same chart, not physical mode probabilities; initializer fractions are not posterior mass'},'new_synthetic_tests':10,'new_optimizations':{'fixed_planes_density':39,'refitted_normals_nominal':3,'training_only_spatial_crossfits':4,'synthetic':4,'total':50},'continuous_runs':0,'filter_updates':0,'qualification':{'AD_RESULT_REPRODUCED':'PASS','PURE_MULTIPLICITY_ISOLATED':'PASS','DENSITY_INVARIANCE':'FAIL','SIGNED_HESSIAN_DECOMPOSITION':'PASS','GEOMETRIC_OBSERVABILITY_SANITY':'FAIL','SPATIAL_NEIGHBORHOOD_ISOLATION':'PASS','ABSOLUTE_POSE_ACCURACY':'NOT_VALIDATED','FRONTEND_RESEARCH_USE':'CONDITIONAL','TRUSTED_LOCALIZATION_FRONTEND':'NOT_VALIDATED','FILTER_POSTERIOR':'FILTER_POSTERIOR_NOT_QUALIFIED','CLOSED_LOOP_RECOMMENDED':'FAIL','PRODUCTION_CANDIDATE':'FAIL'}}
couplings=[]
pose=np.array([0,1,2,6,7,8]);std_pose=np.sqrt(np.diag(P_frozen)[pose])
for root,r in baseline.items():
 rows=[]
 for key,part in r['decomposition']['parts_physical'].items():
  M=np.array(part)[np.ix_(pose,pose)]*np.outer(std_pose,std_pose);off=M-np.diag(np.diag(M))
  rows.append({'term':key,'offdiagonal_prior_standardized_Frobenius':float(np.linalg.norm(off)),'position_attitude_prior_standardized_Frobenius':float(np.linalg.norm(M[:3,3:])),'Z_Roll_prior_standardized_cross':float(M[2,3])})
 couplings.append({'root':root,'parts':rows})
result['cross_coupling_summaries']=couplings
b_unique=[]
for file in (D/'fixed').glob('B_*.json'):
 if file.name.endswith('_frozen.json'):continue
 rr=json.load(open(file));b_unique.extend(int(np.ceil(h['metrics']['unique_targets']/2)) for h in rr['raw']['history'])
rr=load('refit/B_nominal.json');b_unique.extend(int(np.ceil(h['metrics']['unique_targets']/2)) for h in rr['raw']['history'])
result['duplicate_support_guard_audit']={'minimum_original_ID_lower_bound_across_all_B_raw_stages':min(b_unique),'original_minimum_unique_Target_guard':80,'guard_bypass_detected':min(b_unique)<80,'note':'duplicated record IDs are not new original Target identities; lower bound count/2 still passes unchanged support guard at every stored B stage'}
for row in result['frozen_root_density_evaluations']:
 row['unique_original_Target_id_count']=int(row['unique_candidate_target_RECORD_ids']/(2 if row['arm']=='B' else 1))

def convert(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 raise TypeError(type(v).__name__)
(E/'key_results.json').write_text(json.dumps(result,indent=2,default=convert,allow_nan=False)+'\n')
for src,dst in [('prepare.log','AD_REPRODUCTION_LOG.txt'),('density.log','DENSITY_SOLVER_LOG.txt'),('synthetic.log','SYNTHETIC_LOG.txt'),('validation.log','VALIDATION_SOLVER_LOG.txt'),('multiplicity.log','MULTIPLICITY_LOG.txt')]:shutil.copyfile(D/src,E/dst)
fig,axes=plt.subplots(1,3,figsize=(14,3.8));x=np.arange(4);width=.34
for j,root in enumerate(['G1','G2']):
 vals=[r['cost'] for r in fixed if r['root']==root];axes[0].bar(x+(j-.5)*width,np.array(vals)-vals[0],width,label=root)
axes[0].set(xticks=x,xticklabels=['A','B dup2','C half','D local half'],ylabel='energy change from A at SAME pose',title='Fixed original planes');axes[0].legend()
axes[1].bar(np.arange(3),[r['relative_to_original_G1']['translation_difference_norm_m'] for r in nominal]);axes[1].set(xticks=np.arange(3),xticklabels=['B','C','D'],ylabel='metres',title='Nominal reoptimized shift from AD/G1')
axes[2].bar(np.arange(4),[r['truth']['sigma_xyz_curvature_m'][0] for r in sy]);axes[2].axhline(1,color='red',ls='--',label='prior sigma X');axes[2].set(xticks=np.arange(4),xticklabels=['A','dup2','half','halfspace dup'],ylabel='energy-curvature sigma X (m)',title='Exact plane: NO surface X information');axes[2].legend(fontsize=8);fig.tight_layout();fig.savefig(E/'density_and_known_plane.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,4.4));terms=list(dirs[0]['physical_directions'][2]['signed_terms']);xx=np.arange(len(terms))
for j,r in enumerate(dirs):axes[0].bar(xx+(j-.5)*.36,[r['physical_directions'][2]['signed_terms'][key]/1000 for key in terms],.36,label=r['root'])
axes[0].axhline(0,color='black',lw=.5);axes[0].set(xticks=xx,xticklabels=['plane','res cov','cross','num 1st','num 2nd','den 1st','den 2nd','chart','prior'],ylabel='signed Z profile precision / 1000 (1/m^2)',title='Algebraic terms, not independent covariances');axes[0].tick_params(axis='x',labelrotation=45);axes[0].legend()
rs=completed['rows'][:4];axes[1].scatter([r['common_A_training_energy'] for r in rs],[r['fixed_witness_rms'] for r in rs]);
for r in rs:axes[1].annotate(r['label'][-1],(r['common_A_training_energy'],r['fixed_witness_rms']),xytext=(5,3),textcoords='offset points')
axes[1].set(xlabel='SAME A training energy',ylabel='fixed excluded-plane witness RMS (m)',title='Train-only fits; 252 common witnesses');fig.tight_layout();fig.savefig(E/'information_and_validation.png',dpi=170);plt.close(fig)
print('published aggregate summary; all50 fits,39 full-data starts retained; posterior NOT QUALIFIED')
