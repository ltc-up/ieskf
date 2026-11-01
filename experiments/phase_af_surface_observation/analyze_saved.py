from pathlib import Path
import json,numpy as np
from geometry_ieskf.common import dump,C
from geometry_ieskf.surface_observation import Plane,SharedPlaneNoise
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args();D=Path(a.output);p=json.load(open(D/'frozen_private.json'));q=p['prediction']['scores']['prediction_frozen'];g=json.load(open(D/'surface_map_5.0.json'));planes={k:Plane(k,*[np.array(v[n]) for n in ['normal','anchor','tangent','covariance']]) for k,v in g['plane_parameters'].items()};ids=np.array(p['prediction_association']['plane_ids']);J=np.array(q['Jplane']);H=np.array(q['H']);records=[]
for key in np.unique(ids):
 ix=np.flatnonzero(ids==key);N=SharedPlaneNoise(ids[ix],J[ix],planes,C['lidar_sigma_m']**2);I=H[ix].T@N.solve(H[ix]);records.append(dict(key=key,samples=len(ix),Z_information=float(I[2,2]),normal_abs_vertical=float(abs(planes[key].normal[2]))))
Zsum=sum(a['Z_information'] for a in records);largest=max(records,key=lambda a:a['samples']);summary=dict(associated_surface_groups=len(records),largest_sample_group_fraction=largest['samples']/len(ids),largest_sample_group_Z_information_fraction=largest['Z_information']/Zsum,largest_Z_information_fraction=max(a['Z_information'] for a in records)/Zsum,baseline_fit_scatter_rms_quantiles_m=np.percentile([a['scatter_rms'] for a in g['groups']],[0,50,90,100]),baseline_source_residual_above_epsilon_sigma_fraction=float(np.mean(abs(np.array(q['residual']))>C['lidar_sigma_m'])),partition_comparisons=[])
base={int(t):i for i,a in enumerate(g['groups']) for t in a['target_ids']}
for side in [2.5,10.]:
 b=json.load(open(D/('surface_map_'+str(side)+'.json')));other={int(t):i for i,a in enumerate(b['groups']) for t in a['target_ids']};common=base.keys()&other.keys();pairs={(base[t],other[t]) for t in common};left={i:len({j for ii,j in pairs if ii==i}) for i,j in pairs};right={j:len({i for i,jj in pairs if jj==j}) for i,j in pairs};summary['partition_comparisons'].append(dict(side_m=side,common_support_records=len(common),baseline_groups_split_in_other=sum(n>1 for n in left.values()),other_groups_merging_baseline=sum(n>1 for n in right.values()),scope='geometric overlap only, not physical plane ground truth'))
dump(D/'surface_identity_diagnostics.json',dict(summary=summary,private_group_information=records));print('AF saved-factor audit completed')
