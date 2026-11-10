"""Non-overlap raw blocks, explicit past-only forecast time and availability."""
import argparse,json,time
from pathlib import Path
import numpy as np
from geometry_ieskf.velocity_frontend import BlockVelocity
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--noise-audit',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);a=p.parse_args();assert not a.output.exists();d=json.loads(a.input.read_text());R=np.array(json.loads(a.noise_audit.read_text())['noise_proxy']);raw=d['raw_topics']['/sensor/kistler/measurement'];status=d['raw_topics']['/sensor/kistler/status'];j=0;last=None;block=[];bucket=None;events=[];cost=[];used=set();overlap=0
for r in raw:
 while j<len(status) and status[j]['recorder_ns']<=r['recorder_ns']:last=status[j];j+=1
 good=last is not None and last['stst']==0 and 0<=r['header_ns']-last['header_ns']<=250_000_000
 b=r['header_ns']//50_000_000
 if bucket is not None and b!=bucket:
  if len(block)>=6 and 0<r['header_ns']-block[-1][0]['header_ns']<=10_000_000:
   start=time.perf_counter_ns();z,V,info=BlockVelocity.estimate([x[0]['header_ns'] for x in block],[x[0]['raw'] for x in block],r['header_ns'],R);cost.append((time.perf_counter_ns()-start)*1e-6);ids={x[0]['message_id'] for x in block};overlap+=len(ids&used);used|=ids
   events.append(dict(kind='optical',header_ns=r['header_ns'],recorder_ns=r['recorder_ns'],message_id=r['message_id'],velocity=z.tolist(),filtered_velocity=z.tolist(),raw_velocity=r['raw'],variance=V.tolist(),status_good=bool(good and all(x[1] for x in block)),frontend=info|dict(latest_raw_header_ns=block[-1][0]['header_ns'],evaluation_header_ns=r['header_ns'],raw_message_ids=sorted(ids),conditional_variance=np.diag(V).tolist())))
  block=[]
 block.append((r,good));bucket=b
assert overlap==0;a.output.write_text(json.dumps(dict(events=events),separators=(',',':')));a.audit.write_text(json.dumps(dict(events=len(events),raw_used=len(used),raw_ID_overlap=overlap,processing_ms=dict(zip(['p50','p95','p99','max'],np.percentile(cost,[50,95,99,100]))),time='forecast at first next-block raw Header, using only previous block; availability=current raw receipt',common_error_variance=.25,calibration='CONDITIONAL; no duplicate raw IDs, but independence across physical errors not certified'),indent=2)+'\n')
