"""Boundary factor decomposition; no raw identities are published."""
import argparse,json
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.state import inject
from geometry_ieskf.objective import Objective
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
aa,data,contract,e,f,src,prior,P,factor,raw=load_first(D);queries={'original':raw,'nearest_valid':NearestValidCorrespondence(raw.target)};records=[];private=[]
for variant,period in [('original',2),('nearest_valid',5)]:
 folder=D/'phase_ac_geometry_capture'/('nearest_valid' if variant=='nearest_valid' else '');h=json.load(open(folder/'nominal.json'))['history'];zs=[np.array(row['z_after']) for row in h[-period:]];query=queries[variant]
 for edge in range(period):
  z0=zs[edge];direction=zs[(edge+1)%period]-z0;path=json.load(open(O/(variant+'_edge_'+str(edge)+'.json')))
  for bi,b in enumerate(path['boundaries']):
   t=b['boundary_t'];eps=1e-8;zmid=z0+t*direction;corrs=[query.get(inject(prior,z0+(t+sign*eps)*direction),src,factor) for sign in [-1,1]]
   factors=[]
   for c in corrs:
    r,J,rho,rob=Objective(prior,P,c,factor).data(zmid)
    factors.append({int(si):{'target':int(ti),'w':float(w),'rho':float(rr),'r':float(res),'robust_weight':float(hw),'normal':n.tolist()} for si,ti,w,rr,res,hw,n in zip(c['source_index'],c['target_ids'],c['w'],rho,r,rob,c['n'])})
   fa,fb=factors;shared=fa.keys()&fb.keys();entered=fb.keys()-fa.keys();exited=fa.keys()-fb.keys();sw=[i for i in shared if fa[i]['target']!=fb[i]['target']];wc=[i for i in shared if fa[i]['target']==fb[i]['target'] and fa[i]['w']!=fb[i]['w']]
   terms={'entered':sum(fb[i]['w']*fb[i]['rho'] for i in entered),'exited':-sum(fa[i]['w']*fa[i]['rho'] for i in exited),'switched':sum(fb[i]['w']*fb[i]['rho']-fa[i]['w']*fa[i]['rho'] for i in sw),'retained_weight_change':sum(fb[i]['w']*fb[i]['rho']-fa[i]['w']*fa[i]['rho'] for i in wc)}
   angles=[float(np.degrees(np.arccos(np.clip(abs(np.array(fa[i]['normal'])@fb[i]['normal']),0,1)))) for i in sw]
   pop=[]
   for i in sorted(entered|exited):
    target=(fb if i in entered else fa)[i]['target'];dist=[];nearest_valid=[]
    for sign in [-1,1]:
     x=inject(prior,z0+(t+sign*eps)*direction);T=factor.transform(x);y=T[:3,:3]@src[i]+T[:3,3];dist.append(float(np.linalg.norm(y-query.target[target])));dd,tt=query.tree.query(y);nearest_valid.append(bool(query.valid[tt]) if query.cached[tt] else None)
    pop.append({'source_index':i,'old_or_new_target':target,'previous_or_new_target_distance_left_right':dist,'raw_nearest_valid_left_right':nearest_valid,'radius_crossing':bool((dist[0]<=query.c['correspondence_distance_m'])!=(dist[1]<=query.c['correspondence_distance_m']))})
   rec={'variant':variant,'edge':edge,'boundary':bi,'t':t,'source_entered':len(entered),'source_exited':len(exited),'valid_target_switches':len(sw),'same_target_weight_changes':len(wc),'normal_switch_max_deg':max(angles,default=0.),'same_state_jump_terms':terms,'same_state_jump':sum(terms.values()),'measured_one_sided_jump':b['one_sided'][-1]['jump'],'radius_crossings':sum(x['radius_crossing'] for x in pop),'nonnull_same_support_jump':len(entered)==0 and len(exited)==0 and abs(sum(terms.values()))>1e-6}
   records.append(rec);private.append({'record':rec,'left_factors':fa,'right_factors':fb,'population_details':pop})
dump(O/'switch_details_private.json',private);dump(O/'switch_summary.json',records);print('audited boundaries',len(records),'nonzero',sum(abs(r['same_state_jump'])>1e-6 for r in records))
