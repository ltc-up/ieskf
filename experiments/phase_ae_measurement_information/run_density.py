"""Pure fixed-plane density first; refitted normals SECOND, separately labeled."""
import argparse,json
from pathlib import Path
import numpy as np
from case_tools import *
from geometry_ieskf.density_audit import FrozenNormalsView
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
source,prior,P,factor,reference,roots,ad=load_case(D);arms=np.load(O/'density_ids.npz');seeds=json.load(open(O/'seeds.json'));fixed=O/'fixed';fixed.mkdir(exist_ok=True);refit=O/'refit';refit.mkdir(exist_ok=True)
assert not (O/'density_complete.json').exists(),'No repeated sensitivity run'
log=[]
for arm in ['A','B','C','D']:
 q=reference if arm=='A' else FrozenNormalsView(reference,arms[arm]);o=CompactHuberMixture(prior,P,source,factor,q)
 for rn,z in roots.items():dump(fixed/(arm+'_'+rn+'_frozen.json'),metrics(o,z))
 if arm=='A':continue
 for seed in seeds:
  r=run_fit(o,seed,fixed,arm+'_'+seed['id']);log.append({'layer':'fixed_planes','arm':arm,'seed':seed['id'],'status':r['status'],'cost':r['final']['cost']})
 # Drop query/evaluator after all SAME starts for this arm; retain only cached original plane bank.
 del o,q
for arm in ['B','C','D']:
 q=NearestValidCorrespondence(reference.target[arms[arm]]);o=CompactHuberMixture(prior,P,source,factor,q);changes=[]
 for rn,z in roots.items():
  dump(refit/(arm+'_'+rn+'_frozen.json'),metrics(o,z))
  T=factor.transform(inject(prior,z));y=source@T[:3,:3].T+T[:3,3];balls=q.tree.query_ball_point(y,C['correspondence_distance_m']);ids=np.unique(np.concatenate([np.array(v,dtype=int) for v in balls if len(v)]));q.warm_normals(ids);oi=arms[arm][ids];reference.warm_normals(oi);v0=reference.valid[oi];v1=q.valid[ids];both=v0&v1;angles=np.degrees(np.arccos(np.clip(abs(np.einsum('ni,ni->n',reference.normals[oi[both]],q.normals[ids[both]])),0,1)))
  changes.append({'root':rn,'candidate_records':len(ids),'valid_to_invalid':int(np.sum(v0&~v1)),'invalid_to_valid':int(np.sum(~v0&v1)),'jointly_valid_n':int(both.sum()),'normal_angle_quantiles_deg':np.percentile(angles,[50,90,99,100]) if len(angles) else []})
 dump(refit/(arm+'_normal_changes.json'),changes)
 r=run_fit(o,seeds[0],refit,arm+'_nominal');log.append({'layer':'refitted_normals','arm':arm,'seed':'nominal','status':r['status'],'cost':r['final']['cost']})
dump(O/'density_complete.json',log);print('density complete runs',len(log),flush=True)
