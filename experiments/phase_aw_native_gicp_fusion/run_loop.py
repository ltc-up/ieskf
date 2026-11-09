"""Own-state actual matcher feedback around byte-unchanged AR equations.
Only runtime retention and explicit backend covariance callback are replaced.
"""
import argparse,json,gc,types
from pathlib import Path
import numpy as np
from runtime_journal import Journal,journal_json
p=argparse.ArgumentParser()
for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--backend',choices=['ndt_legacy','ndt_fixed','gicp'],required=True);p.add_argument('--profile',action='store_true');p.add_argument('--window',default='lap');p.add_argument('--limit',type=int);a=p.parse_args();assert not a.private.exists();a.private.mkdir(parents=True)
source=Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py';text=source.read_text();old='ledger=[];outputs=[];wall=[]';new="ledger=Journal(a.private/'transactions.jsonl');outputs=Journal(a.private/'states.jsonl');wall=[]";assert text.count(old)==1;text=text.replace(old,new);text=text.replace('wheel_records=[];health_records=[]',"wheel_records=Journal(a.private/'wheel_internal.jsonl',True);health_records=[]")
for key in ['ledger','outputs','wheel_records']:text=text.replace('json.dumps('+key+')','journal_json('+key+')')
needle=' for i,job in enumerate(jobs):';assert text.count(needle)==1;text=text.replace(needle,' gc.collect();gc.freeze()\n'+needle);ns=dict(__name__='aw_frozen_runner',Journal=Journal,journal_json=journal_json,gc=gc);exec(compile(text,str(source),'exec'),ns)
if a.backend in ('gicp','ndt_fixed'):
 def covariance(ndt,components,prior):return np.diag([.5**2 if k in ('x','y','z') else np.deg2rad(1.)**2 for k in components])
 ns['pose_covariance']=covariance
profiler=None
if a.profile:
 from profile_runtime import Timings
 profiler=Timings();profiler.install(ns)
a.mode='F';a.stop_loss=2.;ns['main'](a);r=json.loads(a.output.read_text());r.update(actual_CUDA=(a.backend!='gicp'),actual_CUDA_Deskew=True,actual_small_gicp=(a.backend=='gicp'),backend=a.backend,runtime='streamed journal and frozen immutable heap; GC still enabled',history_horizon_ms=500,own_state_actual_rematching=True,engineered_R_uncalibrated=True);r['pose_statistical_covariance']='UNQUALIFIED_FIXED_0.5M_1DEG' if a.backend in ('gicp','ndt_fixed') else r['pose_statistical_covariance'];a.output.write_text(json.dumps(r,indent=2)+'\n')
if profiler:
 from evaluate import stats
 (a.private/'profile_aggregate.json').write_text(json.dumps({k:stats(v) for k,v in profiler.values.items()},indent=2)+'\n')
