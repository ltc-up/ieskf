"""Read-only attribution of changed active support in the observed cycle."""
import argparse,json,numpy as np
from pathlib import Path
from compare_states import load_first
from geometry_ieskf.common import C,dump
from geometry_ieskf.state import inject
from geometry_ieskf.objective import Objective
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,f,src,prior,P,factor,query=load_first(D);h=json.load(open(O/'nominal.json'))['history'];zs=[np.array(h[-2]['z_after']),np.array(h[-1]['z_after'])];cs=[query.get(inject(prior,z),src,factor) for z in zs];sets=[set(map(int,c['source_index'])) for c in cs];changed=sorted(sets[0]^sets[1]);records=[]
for k in changed:
 samples=[]
 for j,z in enumerate(zs):
  T=factor.transform(inject(prior,z));y=T[:3,:3]@src[k]+T[:3,3];d,ids=query.tree.query(y,k=2);tid=int(ids[0]);normal=query.normals[tid];r=float(normal@(y-query.target[tid]));samples.append({'state':j,'supported':k in sets[j],'nearest_distance_m':float(d[0]),'distance_minus_original_cutoff_m':float(d[0]-C['correspondence_distance_m']),'nearest_target_id':tid,'nearest_target_valid_normal':bool(query.valid[tid]),'next_target_distance_gap_m':float(d[1]-d[0]),'nearest_plane_residual_m':r})
 records.append({'source_index':k,'source_range_m':float(np.linalg.norm(src[k])),'states':samples})
result={'support_counts':[len(c['p']) for c in cs],'changed_source_count':len(changed),'records':records,'original_correspondence_distance_m':C['correspondence_distance_m'],'scope':'two already observed cycle states; no new optimization or data change'};dump(O/'support_boundary_audit.json',result);print(result)
