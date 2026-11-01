"""Evaluate ONE new merit at every old detected discontinuity."""
import argparse,json
from pathlib import Path
import numpy as np
from compare_states import load_first
from geometry_ieskf.common import dump
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.association_objective import CompactHuberMixture
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D);o=CompactHuberMixture(prior,P,source,factor,NearestValidCorrespondence(raw.target));results=[]
for variant,period in [('original',2),('nearest_valid',5)]:
 folder=D/'phase_ac_geometry_capture'/('nearest_valid' if variant=='nearest_valid' else '');h=json.load(open(folder/'nominal.json'))['history'];zs=[np.array(v['z_after']) for v in h[-period:]]
 for edge in range(period):
  start=zs[edge];d=zs[(edge+1)%period]-start;old=json.load(open(O/(variant+'_edge_'+str(edge)+'.json')));rows=[]
  for b in old['boundaries']:
   t=b['boundary_t'];samples=[]
   for eps in [1e-3,1e-4,1e-5,1e-6,1e-7,1e-8]:
    l=start+(t-eps)*d;r=start+(t+eps)*d;cl=o.cost(l);gl=o.gradient(l);cr=o.cost(r);gr=o.gradient(r)
    samples.append({'epsilon':eps,'cost_difference':cr-cl,'directional_gradient_difference':float((gr-gl)@d),'left_gradient':float(gl@d),'right_gradient':float(gr@d),'central_fd_across_old_switch':(cr-cl)/(2*eps),'analytic_mid':float(o.gradient(start+t*d)@d)})
   rows.append({'t':t,'samples':samples})
  results.append({'variant':variant,'edge':edge,'at_old_switches':rows})
dump(O/'smooth_boundary_results.json',results);print('new merit old boundaries',sum(len(r['at_old_switches']) for r in results))
