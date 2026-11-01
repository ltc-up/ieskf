"""Freeze sampling arms and reproduce original AD roots, no refitting."""
import argparse,json
from pathlib import Path
import numpy as np
from case_tools import *
from geometry_ieskf.density_audit import density_arms
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);O.mkdir(exist_ok=True)
assert not (O/'density_ids.npz').exists(),'Do not overwrite frozen sampling definition'
source,prior,P,factor,q,roots,ad=load_case(D);seeds=starting_seeds(P);saved=json.load(open(D/'phase_ac_geometry_capture/predetermined_seeds.json'))
for x,y in zip(seeds,saved):assert x['id']==y['id'] and np.array_equal(x['z'],y['z'])
dump(O/'seeds.json',seeds);o=CompactHuberMixture(prior,P,source,factor,q);reproduce=[]
for row in ad:
 z=np.array(row['z']);value=o.cost(z);g=float(np.max(abs(o.L.T@o.gradient(z))));assert abs(value-row['cost'])<=1e-8
 reproduce.append({'seed_id':row['seed_id'],'cost_delta':value-row['cost'],'gradient_white_inf':g,'saved_gradient_white_inf':row['gradient_white_inf']})
pool=[]
for z in [x['z'] for x in seeds]+list(roots.values()):
 T=factor.transform(inject(prior,z));y=source@T[:3,:3].T+T[:3,3]
 balls=q.tree.query_ball_point(y,C['correspondence_distance_m']);pool.extend(v for v in balls if len(v))
pool=np.unique(np.concatenate(pool));arms,definition=density_arms(q.target,pool,C['source_block_m']);np.savez_compressed(O/'density_ids.npz',**arms);dump(O/'density_definition_private.json',definition)
checks=[]
for name,z in roots.items():
 r=metrics(o,z);old=ad[0 if name=='G1' else 6];oldH=np.array(old['Hessian_probes'][1]['H']);rel=np.linalg.norm(r['decomposition']['H_white_numeric']-oldH)/np.linalg.norm(oldH);assert rel<1e-5
 checks.append({'root':name,'Hessian_relative_repeat_error':rel,'reconstruction_relative':r['decomposition']['reconstruction_relative']});dump(O/('baseline_'+name+'.json'),r)
dump(O/'AD_REPRODUCIBILITY.json',{'roots':checks,'all13_endpoint_rechecks':reproduce,'AD_objective_sha256':sha(Path(__import__('geometry_ieskf.association_objective',fromlist=['x']).__file__)),'arms_counts':{k:len(v) for k,v in arms.items()},'input_binding_sha256':sha(D/'phase_ac_geometry_capture/three_state_binding.json')})
print('AD exact energy repeated13, roots',checks,'arm counts',{k:len(v) for k,v in arms.items()},flush=True)
