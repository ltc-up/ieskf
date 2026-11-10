"""Actual raw-cloud, CUDA Deskew, direct-residual delayed15D closed loop.
Reuses AW streaming and original AR motion/arrival scheduling unchanged.
"""
import argparse,json,gc,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'phase_aw_native_gicp_fusion'))
from runtime_journal import Journal,journal_json
from geometry_ieskf.direct_iterated import DirectIterated,GeometryClient
p=argparse.ArgumentParser()
for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--gaussians',type=Path);p.add_argument('--unregularized-planes',action='store_true');p.add_argument('--shared-frame',action='store_true');p.add_argument('--window',default='lap');p.add_argument('--limit',type=int);p.add_argument('--stop-loss',type=float,default=2.);a=p.parse_args();assert not a.private.exists();a.private.mkdir(parents=True)
s=(Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py').read_text()
s=s.replace('ledger=[];outputs=[];wall=[]',"ledger=Journal(a.private/'transactions.jsonl');outputs=Journal(a.private/'states.jsonl');wall=[]").replace('wheel_records=[];health_records=[]',"wheel_records=Journal(a.private/'wheel_internal.jsonl',True);health_records=[]")
for key in ['ledger','outputs','wheel_records']:s=s.replace('json.dumps('+key+')','journal_json('+key+')')
s=s.replace(' for i,job in enumerate(jobs):',' gc.collect();gc.freeze()\n for i,job in enumerate(jobs):')
old="proc.stdin.write(json.dumps(request)+'\\n');proc.stdin.flush();line=proc.stdout.readline()"
new="request.update(shared_frame=a.shared_frame,unregularized_planes=a.unregularized_planes);request.update(gaussians=str(a.gaussians)) if a.gaussians else None;client=GeometryClient(proc);obs=DirectIterated(lever,client,request);ndt=client.call(request);line=json.dumps(ndt)"
assert s.count(old)==1;s=s.replace(old,new)
s=s.replace("if a.mode!='E' and ndt['converged'] and ndt['candidate_valid']:","if a.mode!='E' and ndt['candidate_valid']:")
s=s.replace("reason='NDT_NOT_CONVERGED_OR_INVALID'","reason='DIRECT_NO_QUALIFIED_FACTORS'")
s=s.replace('Rv=pose_covariance(ndt,components,prior);projection=None','Rv=None;projection=None')
# Ordinary summary calls matching_converged; direct iteration convergence lives in update.
ns=dict(__name__='ax_reuse_ar',Journal=Journal,journal_json=journal_json,gc=gc,GeometryClient=GeometryClient,DirectIterated=DirectIterated)
exec(compile(s,'AX_reuse_AR','exec'),ns);a.mode='F';ns['main'](a)
r=json.loads(a.output.read_text());r.update(actual_CUDA=False,actual_CUDA_Deskew=True,actual_direct_geometry=True,pose_statistical_covariance='NO_POSE_R_SHARED_GEOMETRY_ENGINEERING_MODEL',online_equivalence=False);a.output.write_text(json.dumps(r,indent=2)+'\n')
