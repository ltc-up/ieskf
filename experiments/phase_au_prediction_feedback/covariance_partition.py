"""Separate rear-point prior variance terms without removing cross covariance."""
import argparse,json
from pathlib import Path
import numpy as np
from ndt_global.feedback_diagnostics import pose_variance_components
p=argparse.ArgumentParser();p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();z=np.load(a.private/'canonical_trace.npz');stage=z['stage'];idx=np.flatnonzero(stage==2);P=z['P0'][idx];x=z['before'][idx];lever=np.load(a.private/'initial.npz')['lever'];f=np.load(a.private/'feedback_series.npz');values=[]
for pp,xx in zip(P,x):
 R=xx[6:15].reshape(3,3);values.append(pose_variance_components(pp,R,lever,R[:,1]))
values=np.asarray(values);closure=values.sum(axis=1)-f['prior_lat_variance'];sections=[]
for k in [1,3,9]:
 m=f['corner']==k;parts=values[m].mean(axis=0);sections.append(dict(section=k,mean_position_block_variance=float(parts[0]),mean_attitude_lever_variance=float(parts[1]),mean_signed_cross_variance=float(parts[2]),mean_total_variance=float(parts.sum()),meaning='exact HPH partition, not physical covariance calibration or cross-term removal'))
a.output.write_text(json.dumps(dict(max_closure_m2=float(np.max(abs(closure))),sections=sections,inter_observation_mixing='full-vs-scalar gain comparison conditions on full HPH; not evidence all state correlations are negligible'),indent=2)+'\n');np.save(a.private/'pose_variance_components.npy',values)
