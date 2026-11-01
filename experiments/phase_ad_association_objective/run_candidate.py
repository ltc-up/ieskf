"""Same 13 AC seeds; single frozen coherent objective; no replay/posterior."""
import argparse,json,time
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.association_objective import CompactHuberMixture,solve_mixture
from geometry_ieskf.capture import starting_seeds
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);O.mkdir(exist_ok=True)
aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D);query=NearestValidCorrespondence(raw.target);seeds=starting_seeds(P)
stored=json.load(open(D/'phase_ac_geometry_capture/predetermined_seeds.json'))
for s,t in zip(seeds,stored):assert s['id']==t['id'] and np.array_equal(s['z'],t['z'])
assert not (O/'candidate_results.json').exists(),'No repeated experiment'
results=[]
for s in seeds:
 o=CompactHuberMixture(prior,P,source,factor,query)
 def write(row):
  with open(O/(s['id']+'.iterations.jsonl'),'a') as file:file.write(json.dumps(row,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v.item())+'\n')
  print(s['id'],'stage',row['stage'],row['cost'],row['gradient_white_inf'],row['inner_nit'],row['metrics']['candidate_pairs'],flush=True)
 result=solve_mixture(o,s['z'],on_stage=write);result['seed_id']=s['id']
 # finite-difference FULL dynamic objective in 6 pose-white directions, with fixed full prior.
 from geometry_ieskf.partition import POSE
 Lp=np.linalg.cholesky(P[np.ix_(POSE,POSE)]);tests=[]
 z=result['z'];g=o.gradient(z)
 for j in range(6):
  d=np.zeros(15);d[POSE]=Lp[:,j]
  for eps in [1e-4,1e-5,1e-6]:
   fd=(o.cost(z+eps*d)-o.cost(z-eps*d))/(2*eps);tests.append({'axis':j,'epsilon':eps,'analytic':float(g@d),'fd':fd,'error':abs(fd-g@d)})
 result['endpoint_derivative_tests']=tests
 # candidate pair sets can change harmlessly at k=0; repeat merit/gradient, not hard pair Jaccard, is the certificate.
 neighbors=[]
 for j in range(6):
  d=np.zeros(15);d[POSE]=Lp[:,j]*o.c['convergence_whitened_step']
  for sign in [-1,1]:
   zz=z+sign*d;value,gg,mm,pairs=o.evaluate(zz);neighbors.append({'axis':j,'sign':sign,'cost':value,'gradient_white_inf':float(np.max(abs(o.L.T@gg))),'support':mm})
 result['endpoint_neighborhood']=neighbors
 dump(O/(s['id']+'.json'),result);results.append(result)
 print('FINAL',s['id'],result['status'],result['stages'],result['cost'],result['gradient_white_inf'],result['KKT']['active'],result['wall_ms'],flush=True)
dump(O/'candidate_results.json',results)
