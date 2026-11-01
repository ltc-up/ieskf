"""Endpoint curvature, old-factor mismatch, density sensitivity and atomic diagnostic."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.association_objective import CompactHuberMixture
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.state import inject,chart_map,difference
from geometry_ieskf.capture import assess
from geometry_ieskf.qualification import atomic_select
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D);q=NearestValidCorrespondence(raw.target);r=json.load(open(O/'precision_results.json'));nom=r[0];z=np.array(nom['z']);o=CompactHuberMixture(prior,P,source,factor,q);cost,g,metrics,pairs=o.evaluate(z)
old=[]
for name,query in [('original',raw),('nearest_valid',q)]:
 c=query.get(inject(prior,z),source,factor);obj,m=assess(prior,P,z,c,factor);old.append({'selector':name,'support':c['metrics'],'cost':m['cost'],'gn_white_norm':m['gn_white_norm'],'dual_gradient':m['newton_decrement']})
L=o.L;dist=[]
for row in r:
 zz=np.array(row['z']);dist.append({'seed_id':row['seed_id'],'white_distance_from_nominal':float(np.linalg.norm(np.linalg.solve(L,zz-z))),'cost':row['cost'],'gradient_white_inf':row['gradient_white_inf']})
# Local Laplace ENERGY curvature ONLY; no claimed Gaussian point measurement model.
posts=[];G=chart_map(z)
for hh in nom['Hessian_probes']:
 H=np.array(hh['H']);V=cho_solve(cho_factor(H),L.T);Q=G@(L@V)@G.T;Q=(Q+Q.T)/2
 posts.append({'epsilon':hh['epsilon'],'P':Q,'min_eigen':float(np.linalg.eigvalsh(Q).min()),'sigma_z':float(np.sqrt(Q[2,2])),'sigma_vz':float(np.sqrt(Q[5,5]))})
candidate=inject(prior,z);decision={'accepted':True,'status':'ISOLATED_LAPLACE_ENERGY_DIAGNOSTIC','reasons':[]}
x,Q,decision=atomic_select(prior,P,candidate,np.array(posts[1]['P']),decision)
rollback_x,rollback_P,rollback_decision=atomic_select(prior,P,candidate,np.array(posts[1]['P']),{'accepted':False,'status':'NO_RECURSIVE_POSTERIOR_QUALIFICATION','reasons':['NEW_ENERGY_INFORMATION_UNCALIBRATED']})
relative=[float(np.linalg.norm(np.array(p['P'])-np.array(posts[1]['P']))/np.linalg.norm(posts[1]['P'])) for p in posts]
result={'prior':{'sigma_z':float(np.sqrt(P[2,2])),'sigma_vz':float(np.sqrt(P[5,5]))},'isolated_energy_laplace_posts':posts,'relative_P_difference':relative,'atomic_diagnostic':decision,'state_error':float(np.max(abs(difference(x,prior)-z))),'P_matches':bool(np.array_equal(Q,np.array(posts[1]['P']))),'rollback_exact':bool(np.array_equal(rollback_P,P) and np.max(abs(difference(rollback_x,prior)))==0),'covariance_updates_in_filter':0,'continuous_runs':0,'old_objectives_at_mixture_root':old,'endpoint_distances':dist}
dump(O/'endpoint_evidence.json',result);print('root groups',dist);print('Laplace sigmaz',[p['sigma_z'] for p in posts],'P relative',relative,'atomic',result['state_error'],result['rollback_exact'])
