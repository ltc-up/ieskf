"""Reuse actual probe solutions; common-axis matrix stability, not fixed-pose FD."""
import argparse,json
from pathlib import Path
import numpy as np
from ndt_global.matcher_geometry import ndt_native_information,gicp_information,compare
p=argparse.ArgumentParser();p.add_argument('--probes',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=[]
for r in [json.loads(l) for l in a.probes.open()]:
 v=dict(corner=r['corner'],candidate_set_correctness='UNKNOWN',evaluated_at_actual_converged_or_returned_poses=True)
 for name,fn in [('ndt',ndt_native_information),('gicp',gicp_information)]:
  center=r['central_'+name];R=np.array(center['final'])[:3,:3];I=fn(center);results=[]
  for probe in r['probes']:
   candidate=probe[name];Rn=np.array(candidate['final'])[:3,:3];Q=np.zeros((6,6));Q[:3,:3]=Rn.T@R;Q[3:,3:]=Rn.T@R;J=Q.T@fn(candidate)@Q;z=compare(I,J);results.append(dict(converged=candidate['converged'],shape_cosine=z['trace_shape_cosine'],weak_axis_abs_cosine=z['weakest_axis_abs_cosine'],relative_information_change=float(np.linalg.norm(J-I)/np.linalg.norm(I))))
  v[name]=results
 out.append(v)
a.output.write_text(json.dumps(dict(cases=out,pose_chart_transport='all directions transported to the central body axes; same base reference',not_fixed_pose_FD=True,not_statistical_covariance=True),indent=2)+'\n')
