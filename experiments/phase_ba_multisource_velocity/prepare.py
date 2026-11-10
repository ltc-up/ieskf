"""Past-only preprocessing; no GNSS, trajectory, fitted lag or geography input."""
import argparse,json,copy,time
from pathlib import Path
import numpy as np
from geometry_ieskf.velocity_frontend import PolynomialVelocity
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);p.add_argument('--legacy',action='store_true');a=p.parse_args();assert not a.output.exists();d=json.loads(a.input.read_text());raw=d['raw_topics']['/sensor/kistler/measurement'];filters=[PolynomialVelocity(a.legacy) for _ in range(2)];j=0;events=[];traces=[];push_cost=[];query_cost=[]
for e in d['events']:
 while j<len(raw) and raw[j]['recorder_ns']<=e['recorder_ns'] and raw[j]['header_ns']<=e['header_ns']:
  r=raw[j];tick=time.perf_counter_ns()
  for k in range(2):filters[k].push(r['header_ns']*1e-9,r['raw'][k])
  push_cost.append((time.perf_counter_ns()-tick)*1e-6);j+=1
 out=copy.deepcopy(e);v=[];var=[];tick=time.perf_counter_ns()
 for f in filters:
  x,P=f.query(e['header_ns']*1e-9);v.append(float(x[0]));var.append(float(P[0,0]))
 query_cost.append((time.perf_counter_ns()-tick)*1e-6)
 out['raw_velocity']=e['velocity'];out['filtered_velocity']=v;out['velocity']=v;out['variance']=(np.asarray(e['variance'])+np.diag(var)).tolist();out['frontend']=dict(evaluation_header_ns=e['header_ns'],latest_raw_header_ns=raw[j-1]['header_ns'],conditional_variance=var,correlated_output=True);events.append(out);traces.append([e['header_ns'],*e['velocity'],*v,*var])
a.output.write_text(json.dumps(dict(events=events),separators=(',',':')));r=np.array(traces);np.savez_compressed(a.audit.with_suffix('.npz'),frontend=r);a.audit.write_text(json.dumps(dict(push_pair_ms_quantiles=np.percentile(push_cost,[50,95,99,100]).tolist(),query_pair_ms_quantiles=np.percentile(query_cost,[50,95,99,100]).tolist(),events=len(events),raw_used=j,filter='legacy_integer_Q' if a.legacy else 'PSD_white_jerk',raw_minus_filtered_rms=np.sqrt(np.mean((r[:,1:3]-r[:,3:5])**2,axis=0)).tolist(),mean_conditional_variance=np.mean(r[:,5:7],axis=0).tolist(),time='causal query at actual Header; latest contributing sample Header and receipt not later',noise='original engineeringR common-error floor plus conditional frontend variance, not calibrated; overlapping filter outputs correlated'),indent=2)+'\n')
