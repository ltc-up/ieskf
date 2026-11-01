"""Inspect an observed alternating active-set cycle, without new optimization."""
import argparse,json,numpy as np
from pathlib import Path
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.state import inject
from geometry_ieskf.objective import Objective
from geometry_ieskf.capture import assess
from geometry_ieskf.qualification import pair_retention
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,frame,src,prior,P,factor,query=load_first(D);nom=json.load(open(O/'nominal.json'));h=nom['history'];zs=[np.array(h[-2]['z_after']),np.array(h[-1]['z_after'])];cs=[query.get(inject(prior,z),src,factor) for z in zs];obs=[]
for i,c in enumerate(cs):
 o=Objective(prior,P,c,factor)
 for j,z in enumerate(zs):
  obj,m=assess(prior,P,z,c,factor);obs.append({'factor_set':i,'state':j,'cost':o.cost(z),'gn_white_norm':m['gn_white_norm'],'dual_gradient':m['newton_decrement'],'gradient_white_norm':m['gradient_white_norm']})
pa=pair_retention(*cs);maps=[{int(k):i for i,k in enumerate(c['source_index'])} for c in cs];common=maps[0].keys()&maps[1].keys();changed=[k for k in sorted(common) if cs[0]['target_ids'][maps[0][k]]!=cs[1]['target_ids'][maps[1][k]]];events=[]
for k in changed:
 ids=[int(c['target_ids'][m[k]]) for c,m in zip(cs,maps)];norms=[c['n'][m[k]] for c,m in zip(cs,maps)];detailed=[]
 for i,z in enumerate(zs):
  T=factor.transform(inject(prior,z));y=T[:3,:3]@src[k]+T[:3,3];ds=[float(np.linalg.norm(y-query.target[t])) for t in ids];r=[float(n@(y-query.target[t])) for n,t in zip(norms,ids)];detailed.append({'state':i,'euclidean_distance_to_two_targets':ds,'signed_distance_difference_target0_minus_target1':ds[0]-ds[1],'point_plane_residual_to_two_planes':r})
 events.append({'source_index':k,'target_ids':ids,'absolute_normal_angle_degrees':float(np.degrees(np.arccos(np.clip(abs(norms[0]@norms[1]),0,1)))),'target_multiplicities':[int(np.sum(c['target_ids']==t)) for c,t in zip(cs,ids)],'values':detailed})
L=np.linalg.cholesky(P);repeat2=[float(np.linalg.norm(np.linalg.solve(L,np.array(h[j]['z_after'])-np.array(h[j-2]['z_after'])))) for j in range(max(2,len(h)-10),len(h))];repeat1=[float(np.linalg.norm(np.linalg.solve(L,np.array(h[j]['z_after'])-np.array(h[j-1]['z_after'])))) for j in range(max(1,len(h)-10),len(h))]
# Compare correspondence-set objectives at exactly the SAME state, isolating re-selection.
losses=[]
for i,c in enumerate(cs):
 o=Objective(prior,P,c,factor);r,J,rho,rob=o.data(zs[0]);losses.append({int(k):float(w*v) for k,w,v in zip(c['source_index'],c['w'],rho)})
diff=[{'source_index':k,'loss_delta_set1_minus_set0':losses[1].get(k,0)-losses[0].get(k,0)} for k in sorted(losses[0].keys()|losses[1].keys())];diff.sort(key=lambda x:abs(x['loss_delta_set1_minus_set0']),reverse=True)
result={'factor_state_matrix':obs,'pairs':pa,'changed_sources':events,'source_population_difference':len(set(maps[0])^set(maps[1])),'max_two_step_repeat_white_distance':max(repeat2),'min_one_step_white_distance':min(repeat1),'max_one_step_white_distance':max(repeat1),'same_state_reassociation_geometry_cost_jump':sum(x['loss_delta_set1_minus_set0'] for x in diff),'top_loss_changes':diff[:10],'last10_inner_success':all(x['inner_success'] for x in h[-10:]),'last10_step_scales':[x['line_scale'] for x in h[-10:]],'last_original_guard_active':nom['final_original_guard_KKT']['active']}
dump(O/'cycle_audit.json',result);print(result)
