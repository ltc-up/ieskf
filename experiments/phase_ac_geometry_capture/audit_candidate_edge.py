"""Inspect remaining source-domain and valid-plane switches on the candidate cycle."""
import argparse,json,numpy as np
from pathlib import Path
from compare_states import load_first
from geometry_ieskf.common import C,dump
from geometry_ieskf.state import inject
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,f,src,prior,P,factor,old=load_first(D);q=NearestValidCorrespondence(old.target);h=json.load(open(O/'nearest_valid/nominal.json'))['history'];zs=[np.array(x['z_after']) for x in h[-2:]];cs=[q.get(inject(prior,z),src,factor) for z in zs];maps=[{int(k):j for j,k in enumerate(c['source_index'])} for c in cs];exit_ids=sorted(maps[0].keys()-maps[1].keys());removed=[]
for k in exit_ids:
 tid=int(cs[0]['target_ids'][maps[0][k]]);ds=[]
 for z in zs:
  T=factor.transform(inject(prior,z));y=T[:3,:3]@src[k]+T[:3,3];ds.append(float(np.linalg.norm(y-q.target[tid])))
 removed.append({'source_index':k,'previous_valid_target':tid,'distance_to_previous_valid_target_m':ds,'previous_target_still_normal_valid':bool(q.valid[tid])})
result={'source_exits':len(exit_ids),'original_radius_m':C['correspondence_distance_m'],'all_exited_sources_cross_radius_of_previous_valid_target':bool(all(v['distance_to_previous_valid_target_m'][0]<=C['correspondence_distance_m']<v['distance_to_previous_valid_target_m'][1] for v in removed)),'details':removed,'scope':'one already observed candidate-cycle edge; no search radius or filter change'};dump(O/'candidate_edge_audit.json',result);print(result)
