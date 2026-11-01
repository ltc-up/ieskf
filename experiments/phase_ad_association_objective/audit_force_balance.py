"""Separate plane residual and association-kernel derivatives in prior-white units."""
import argparse,json
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump,jr
from geometry_ieskf.association_objective import CompactHuberMixture
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.state import inject
from geometry_ieskf.partition import POSE,OTHER
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D);q=NearestValidCorrespondence(raw.target);rows=json.load(open(O/'precision_results.json'));results=[]
for j in [0,6]:
 row=rows[j];z=np.array(row['z']);o=CompactHuberMixture(prior,P,source,factor,q);cost,g,m,pairs=o.evaluate(z);gy=np.zeros_like(source);force=pairs['responsibility'][:,None]*np.clip(pairs['residual'],-o.c['huber_delta_m'],o.c['huber_delta_m'])[:,None]*pairs['normals']/o.c['lidar_sigma_m']**2
 for k in range(3):gy[:,k]=np.bincount(pairs['source_index'],weights=force[:,k],minlength=len(source))
 plane=np.zeros(15);plane[:3]=gy.sum(0);x=inject(prior,z);plane[6:9]=np.cross(source-factor.lever,gy@x.R).sum(0)@jr(z[6:9]);prior_g=o.Q.T@(o.Q@z);kernel=g-prior_g-plane
 K=np.linalg.solve(P[np.ix_(POSE,POSE)],P[np.ix_(OTHER,POSE)].T).T
 results.append({'seed_id':row['seed_id'],'prior_white_gradient_norm':float(np.linalg.norm(o.L.T@prior_g)),'plane_residual_white_gradient_norm':float(np.linalg.norm(o.L.T@plane)),'kernel_association_white_gradient_norm':float(np.linalg.norm(o.L.T@kernel)),'total_white_gradient_norm':float(np.linalg.norm(o.L.T@g)),'conditional_other_recovery_error':float(np.max(abs(z[OTHER]-K@z[POSE]))),'full_state_component_clipped':False})
dump(O/'force_balance.json',results);print(results)
