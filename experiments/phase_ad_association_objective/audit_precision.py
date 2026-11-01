"""Uniform full15 Newton stationarity probe; no posterior or replay."""
import argparse,json,time
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.association_objective import CompactHuberMixture
from geometry_ieskf.partition import margins
from geometry_ieskf.capture import step_margins
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D);query=NearestValidCorrespondence(raw.target);old=json.load(open(O/'candidate/candidate_results.json'));results=[]
for row in old:
 start=time.perf_counter();o=CompactHuberMixture(prior,P,source,factor,query);z=np.array(row['z']);iterations=[]
 def Hess(z,eps):
  u=np.linalg.solve(o.L,z);cols=[]
  for j in range(15):
   d=np.eye(15)[j]*eps;cols.append(o.L.T@(o.gradient(o.L@(u+d))-o.gradient(o.L@(u-d)))/(2*eps))
  H=np.array(cols).T;return (H+H.T)/2,float(np.max(abs(H-H.T)))
 for k in range(3):
  c=o.cost(z);g=o.L.T@o.gradient(z);H,asym=Hess(z,1e-5);ev=np.linalg.eigvalsh(H)
  if ev.min()<=0:iterations.append({'reason':'NON_POSITIVE_HESSIAN','eigenvalues':ev});break
  if np.max(abs(g))<=o.c['convergence_whitened_step']:break
  du=np.linalg.solve(H,-g);dz=o.L@du;used=None;actual=None
  for alpha in o.c['line_search_scales']:
   trial=z+alpha*dz
   if min(margins(trial))<0 or min(step_margins(trial,z))<0:continue
   value=o.cost(trial);newg=o.L.T@o.gradient(trial)
   if value<=c+1e-10 and np.max(abs(newg))<np.max(abs(g)):
    used=alpha;actual=value-c;z=trial;break
  iterations.append({'iteration':k,'gradient_inf_before':float(np.max(abs(g))),'Newton_white_norm':float(np.linalg.norm(du)),'predicted_change':float(g@du+.5*du@H@du),'actual_cost_change':actual,'alpha':used,'Hessian_asymmetry_inf':asym,'eigenvalues':ev})
  if used is None:break
 hs=[]
 for eps in [1e-4,1e-5,1e-6]:
  H,asym=Hess(z,eps);hs.append({'epsilon':eps,'H':H,'asymmetry_inf':asym,'eigenvalues':np.linalg.eigvalsh(H)})
 value,g,m,pairs=o.evaluate(z);white=o.L.T@g
 out={'seed_id':row['seed_id'],'raw_status':row['status'],'raw_gradient_inf':row['gradient_white_inf'],'status':'INTERIOR_STATIONARY_NUMERIC_PROBE' if np.max(abs(white))<=o.c['convergence_whitened_step'] and min(margins(z))>1e-5 and m['support_good'] and all(v['eigenvalues'].min()>0 for v in hs) else 'NOT_QUALIFIED','z':z,'cost':value,'gradient_white_inf':float(np.max(abs(white))),'margins':margins(z),'metrics':m,'iterations':iterations,'Hessian_probes':hs,'wall_ms':1000*(time.perf_counter()-start),'covariance_updates':0}
 dump(O/('precision_'+row['seed_id']+'.json'),out);results.append(out);print(row['seed_id'],out['status'],out['gradient_white_inf'],out['cost'],out['wall_ms'],flush=True)
dump(O/'precision_results.json',results)
