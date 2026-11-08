"""Three-order real timing fixture. Recorded fused odom is NOT independent odom.
No acceptance/convergence/covariance is inferred from a public pose topic.
"""
import csv,json,argparse,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,PoseHistory

def transform(r):
 T=np.eye(4);T[:3,3]=[float(r[k]) for k in ('x','y','z')];T[:3,:3]=Rotation.from_quat([float(r[k]) for k in ('qx','qy','qz','qw')]).as_matrix();return T

def summarize(v):
 return dict(zip(('p50','p95','p99','max'),np.percentile(v,[50,95,99,100]).tolist())) if v else None

def run(root):
 od=list(csv.DictReader(open(root/'ekf.csv')));nd=list(csv.DictReader(open(root/'ndt.csv')))
 contract=Contract('TIMING_FIXTURE_ONLY','TIMING_FIXTURE_ONLY','fixture')
 def ns(r,k='stamp'):return int(r[k+'_sec'])*10**9+int(r[k+'_nsec'])
 # Header-sorted oracle interpolation is a temporal reference, not physical truth.
 # Direct oracle brackets avoid retaining the whole run in the bounded online cache.
 stamps=np.array([ns(r) for r in od]);order=np.argsort(stamps);stamps=stamps[order];Ts=[transform(od[i]) for i in order]
 def oracle(t):
  from bisect import bisect_left
  i=bisect_left(stamps,t)
  if i==0 or i==len(stamps):raise ValueError('NO_BRACKET')
  hh=PoseHistory(contract);hh.add(int(stamps[i-1]),Ts[i-1],contract);hh.add(int(stamps[i]),Ts[i],contract);return hh.at(t)
 hist=PoseHistory(contract);latest=None
 events=[(ns(r,'receive'),0,r) for r in od]+[(ns(r,'receive'),1,r) for r in nd];events.sort(key=lambda x:(x[0],x[1]))
 errors=[];comp=[];reasons={};delay=[];header=[];wall=[]
 for recv,kind,r in events:
  t=ns(r)
  if kind==0:
   try:hist.add(t,transform(r),contract);latest=transform(r)
   except ValueError as e:reasons[str(e)]=reasons.get(str(e),0)+1
  else:
   tick=time.perf_counter_ns();delay.append((recv-t)/1e6)
   try:
    O=oracle(t);M=transform(r);expected=M@np.linalg.inv(O);header.append(0.)
    if latest is not None:errors.append(float(np.linalg.norm((M@np.linalg.inv(latest))[:3,3]-expected[:3,3])))
    if recv-t>500_000_000:raise ValueError('STALE')
    corrected=M@np.linalg.inv(hist.at(t));comp.append(float(np.max(abs(corrected-expected))))
   except ValueError as e:reasons[str(e)]=reasons.get(str(e),0)+1
   wall.append((time.perf_counter_ns()-tick)/1e6)
 return dict(real_odom_rows=len(od),real_ndt_rows=len(nd),header_sorted_pairings=len(header),arrival_latest_pairings=len(errors),delayed_historical_pairings=len(comp),arrival_latest_correction_translation_difference_m=summarize(errors),delay_compensated_vs_header_max_matrix_difference=max(comp,default=None),receipt_minus_header_ms=summarize(delay),rejections=reasons,diagnostic_wall_ms=summarize(wall),qualification='TIMING_DIAGNOSTIC_ONLY',local_delayed_15D_update='SEPARATE_OFFLINE_CHECKPOINT_PROTOTYPE; see local_timing_results.json',actual_callback_order='UNVERIFIED: recorder receive order only',online_equivalence=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(run(a.dataset),indent=2)+'\n')
