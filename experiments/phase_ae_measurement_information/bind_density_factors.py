"""Record exact private factors and verify pure multiplicity algebra at frozen roots."""
import argparse,json
from pathlib import Path
import numpy as np
from case_tools import *
from geometry_ieskf.density_audit import FrozenNormalsView
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);B=O/'factor_bindings';B.mkdir(exist_ok=True)
source,prior,P,factor,reference,roots,ad=load_case(D);arms=np.load(O/'density_ids.npz');records=[]
for name,z in roots.items():
 base=CompactHuberMixture(prior,P,source,factor,reference);value,g,metrics,original=base.evaluate(z);si=original['source_index'];ids=original['target_ids'];k=original['kernel'];rho=original['rho'];tau=C['huber_delta_m']**2;R=C['correspondence_distance_m'];h=C['huber_delta_m'];cap=.5*R*R if R<=h else h*(R-.5*h);prior_cost=.5*np.linalg.norm(np.linalg.solve(base.L,z))**2
 for arm in ['A','B','C','D']:
  multiplicity=np.bincount(arms[arm],minlength=len(reference.target));m=multiplicity[ids];den=1+np.bincount(si,weights=m*k,minlength=len(source));num=np.exp(-cap/tau)+np.bincount(si,weights=m*k*np.exp(-rho/tau),minlength=len(source));algebra=float(prior_cost-tau*np.log(num/den).sum()/C['lidar_sigma_m']**2)
  q=reference if arm=='A' else FrozenNormalsView(reference,arms[arm]);o=CompactHuberMixture(prior,P,source,factor,q);materialized,gg,mm,pairs=o.evaluate(z)
  assert abs(algebra-materialized)<1e-8
  np.savez_compressed(B/(arm+'_'+name+'.npz'),source_ids=pairs['source_index'],target_record_ids=pairs['target_ids'],original_target_ids=arms[arm][pairs['target_ids']],normals=pairs['normals'],residual=pairs['residual'],rho=pairs['rho'],kernel=pairs['kernel'],responsibility=pairs['responsibility'],loss_per_source=pairs['loss_per_source'])
  records.append({'root':name,'arm':arm,'algebra_energy':algebra,'materialized_energy':materialized,'difference':algebra-materialized,'factor_n':len(pairs['source_index'])})
dump(O/'multiplicity_algebra.json',records);print('all8 algebra versus materialization checks max difference',max(abs(r['difference']) for r in records))
