"""Recover committed JSONL prefix without claiming the requested run completed."""
import argparse,json,collections
from pathlib import Path
import numpy as np
from ndt_global.ar_replay import observation_health
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--history',type=float,required=True);a=p.parse_args();tx=[json.loads(v) for v in (a.run/'transactions.jsonl').read_text().splitlines()];states=[json.loads(v) for v in (a.run/'states.jsonl').read_text().splitlines()];assert len(tx)==len(states);last=None;health=[]
for r in tx:
 if r['update']['accepted']:last=r['ref']
 s=observation_health(r['ref'],last,tx[0]['ref'])
 if r['update']['accepted'] and r['ndt'].get('av_projection',{}).get('observed_rank',6)<6:s='PARTIAL_CONSTRAINT'
 health.append(dict(header_ns=r['ref'],state=s))
for name,v in [('transactions',tx),('states',states),('health',health)]:
 path=a.run/(name+'.json');assert not path.exists();path.write_text(json.dumps(v))
age=np.array([(r['complete']-r['ref'])*1e-6 for r in tx]);out=dict(requested_frames=4134,processed_frames=len(tx),requested_complete=False,duration_s=(tx[-1]['ref']-tx[0]['ref'])*1e-9,accepted=sum(r['update']['accepted'] for r in tx),stop_reason='DESKEW_HISTORY_COVERAGE_ASSERTION',history_seconds=a.history,startup_ms=None,wall_ms=dict(qualification='NOT_SAVED_BY_INTERRUPTED_RUN; do not fabricate full transaction wall time'),model_completion_age_ms=dict(p50=float(np.median(age)),p95=float(np.percentile(age,95)),p99=float(np.percentile(age,99)),max=float(age.max()),over500ms=int(sum(age>500)),over2000ms=int(sum(age>2000))),posthoc_stream_archive=True,actual_CUDA=True,algorithm='FROZEN_V3',live_summary_missing=True,GNSS_used=False)
a.output.write_text(json.dumps(out,indent=2)+'\n')
