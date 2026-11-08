"""Evaluate a pre-existing external window; never select by estimator success."""
import argparse,json
from pathlib import Path
import numpy as np
from ndt_global.validation import percentiles

def run(a):
 # Window identity is provided by a prior artifact, not fit to this estimator.
 fields={}
 for line in (a.prior/'lap_window.txt').read_text().splitlines():
  if '=' in line:k,v=line.split('=',1);fields[k]=v
 lo=round(float(fields['LAP_START'])*1e9);hi=round(float(fields['LAP_END'])*1e9);d=json.loads(a.input.read_text());ts=np.load(a.local/'local_stamps_ns.npy');r=np.load(a.local/'local_trace.npy',mmap_mode='r');ids=np.flatnonzero((ts>=lo)&(ts<=hi));rows=r[ids];ev=[e for e in d['events'] if lo<=e['header_ns']<=hi];counts={k:sum(e['kind']==k for e in ev) for k in ['imu','wheel','pose','ndt','gps','gps2']}
 return dict(window_origin='pre-existing GNSS-derived lap_window artifact; discovered after whole-record execution, not an estimator-selected successful window',duration_s=(hi-lo)/1e9,lap_certificate='NOT_SURVEYED',input_counts=counts,local_output_count=len(ids),local_output_span_s=(int(ts[ids[-1]])-int(ts[ids[0]]))/1e9 if len(ids) else 0.,local_Z_span_m=float(np.ptp(rows[:,3])) if len(ids) else None,local_vz_m_s=percentiles(rows[:,6]) if len(ids) else None,accuracy_reference_qualified=False,local_state_reinitialized=False,complete_local_window_processed=bool(len(ids) and ts[0]<=lo and ts[-1]>=hi),trusted_global_output=0,new_causal_NDT_frames=0,qualification='NO_GO')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prior',type=Path,required=True);p.add_argument('--input',type=Path,required=True);p.add_argument('--local',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(run(a),indent=2)+'\n')
