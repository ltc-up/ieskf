"""Reuse published native-time evaluation; joint16 calibration remains in filter.
Only reported physical-state15 gain slice is selected; no algorithm data changes.
"""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser()
for k in ['run','baseline','motion','as-evaluation','at-bindings','run-summary','private','output']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--matching-backend',choices=['ndt','gicp'],default='ndt');p.add_argument('--reference-cache',type=Path);p.add_argument('--geometry',type=Path);p.add_argument('--label',required=True);p.add_argument('--expected',type=int,required=True);a=p.parse_args()
source=Path(__file__).parents[1]/'phase_aw_native_gicp_fusion/evaluate.py';s=source.read_text();s=s.replace("np.asarray(r['update']['K'])","np.asarray(r['update']['K'])[:15]");ns=dict(__name__='ay_frozen_reference');exec(compile(s,str(source),'exec'),ns);ns['evaluate'](a);r=json.loads(a.output.read_text());r['lateral_gain_semantics']='physical15 subresponse of actual joint filter; calibration cross covariance retained';a.output.write_text(json.dumps(r,indent=2)+'\n')
