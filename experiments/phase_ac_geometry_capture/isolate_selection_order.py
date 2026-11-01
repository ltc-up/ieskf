"""Same-state association-only counterfactual for the previously identified point."""
import argparse,json,numpy as np
from pathlib import Path
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.state import inject
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,f,src,prior,P,factor,old=load_first(D);new=NearestValidCorrespondence(old.target);cycle=json.load(open(O/'nominal.json'));identified=json.load(open(O/'support_boundary_audit.json'))['records'][0]['source_index'];records=[]
for i,h in enumerate(cycle['history'][-2:]):
 z=np.array(h['z_after']);x=inject(prior,z);row={'cycle_state':i}
 for name,q in [('original',old),('nearest_valid',new)]:
  c=q.get(x,src,factor);ix=np.flatnonzero(c['source_index']==identified);row[name]={'focal_supported':bool(len(ix)),'focal_target_id':int(c['target_ids'][ix[0]]) if len(ix) else None,'support_n':len(c['p'])}
 records.append(row)
dump(O/'selection_order_isolation.json',records);print(records)
