"""Direct old-selector continuity/derivative audit. Raw evidence stays local."""
import argparse,json,time
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.objective import Objective
from geometry_ieskf.state import inject
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.qualification import pair_retention
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);O.mkdir(exist_ok=True)
aa,data,contract,e,f,source,prior,P,factor,orig=load_first(D);valid=NearestValidCorrespondence(orig.target);L=np.linalg.cholesky(P)
results=[];start=time.perf_counter()
for name,query,period,folder in [('original',orig,2,D/'phase_ac_geometry_capture'),('nearest_valid',valid,5,D/'phase_ac_geometry_capture/nearest_valid')]:
 h=json.load(open(folder/'nominal.json'))['history'];zs=[np.array(x['z_after']) for x in h[-period:]]
 for edge in range(period):
  z0=zs[edge];d=zs[(edge+1)%period]-z0;rows=[];cache={}
  def at(t):
   key=float(t)
   if key not in cache:
    z=z0+t*d;c=query.get(inject(prior,z),source,factor);o=Objective(prior,P,c,factor);r,J,rho,rob=o.data(z)
    row={'t':float(t),'cost':o.cost(z),'prior':float(.5*np.linalg.norm(np.linalg.solve(L,z))**2),'directional_frozen_gradient':float(o.gradient(z)@d),'support':c['metrics']['support_n'],'source_ids':c['source_index'],'target_ids':c['target_ids'],'weights':c['w'],'residual':r,'Huber':rho,'robust_weights':rob,'normals':c['n']}
    cache[key]=(row,c,o)
   return cache[key]
  ts=np.linspace(0,1,33)
  def signature(c):return (c['source_index'].tobytes(),c['target_ids'].tobytes(),c['w'].tobytes())
  boundaries=[];interior=[]
  for t in ts:rows.append(at(t)[0])
  for left,right in zip(ts[:-1],ts[1:]):
   rl,cl,ol=at(left);rr,cr,orr=at(right)
   if signature(cl)==signature(cr):
    mid=(left+right)/2;row,c,o=at(mid);eps=1e-5
    fd=(at(mid+eps)[0]['cost']-at(mid-eps)[0]['cost'])/(2*eps)
    interior.append({'t':mid,'analytic':row['directional_frozen_gradient'],'central_fd':fd,'error':abs(fd-row['directional_frozen_gradient'])});continue
   lo,hi=left,right;sig=signature(cl)
   for _ in range(40):
    mid=(lo+hi)/2
    if signature(at(mid)[1])==sig:lo=mid
    else:hi=mid
   b=(lo+hi)/2;one=[]
   for eps in [1e-3,1e-4,1e-5,1e-6,1e-7,1e-8]:
    lm,lc,loo=at(b-eps);rm,rc,roo=at(b+eps)
    sameLeft=signature(at(b-2*eps)[1])==signature(lc);sameRight=signature(at(b+2*eps)[1])==signature(rc)
    one.append({'epsilon':eps,'left_cost':lm['cost'],'right_cost':rm['cost'],'jump':rm['cost']-lm['cost'],'left_analytic':lm['directional_frozen_gradient'],'right_analytic':rm['directional_frozen_gradient'],'left_fd':(lm['cost']-at(b-2*eps)[0]['cost'])/eps,'right_fd':(at(b+2*eps)[0]['cost']-rm['cost'])/eps,'left_stable':sameLeft,'right_stable':sameRight,'support_left':lm['support'],'support_right':rm['support'],'pair':pair_retention(lc,rc),'cross_fixed_left_cost_at_right':loo.cost(z0+(b+eps)*d),'cross_fixed_right_cost_at_left':roo.cost(z0+(b-eps)*d)})
   boundaries.append({'boundary_t':b,'bracket_width':hi-lo,'one_sided':one})
  # Direction of the actual AC step; small steps retain an active cell; finite steps can cross it.
  z=z0;c=query.get(inject(prior,z),source,factor);o=Objective(prior,P,c,factor)
  descent=[{'alpha':alpha,'dynamic_change':at(alpha)[0]['cost']-at(0)[0]['cost'],'fixed_change':o.cost(z0+alpha*d)-o.cost(z0),'pairs':pair_retention(c,at(alpha)[1])} for alpha in [1e-6,1e-4,.01,.1,1.]]
  result={'variant':name,'edge':edge,'white_length':float(np.linalg.norm(np.linalg.solve(L,d))),'rows':rows,'boundaries':boundaries,'smooth_interiors':interior,'step_direction':descent}
  dump(O/(name+'_edge_'+str(edge)+'.json'),result)
  results.append({k:v for k,v in result.items() if k!='rows'});print(name,edge,'boundaries',len(boundaries),'interiors',len(interior),'endpoint_change',at(1)[0]['cost']-at(0)[0]['cost'],flush=True)
dump(O/'path_summary_private.json',{'edges':results,'wall_s':time.perf_counter()-start})
