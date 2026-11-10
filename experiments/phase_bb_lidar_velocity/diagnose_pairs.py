"""Frozen raw-Deskew pair diagnostics; never an own-state closed-loop claim."""
import argparse,json,subprocess,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--transactions',type=Path,required=True);p.add_argument('--meta',type=Path,required=True);p.add_argument('--matcher',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--limit',type=int);a=p.parse_args();assert not a.out.exists();tx=json.loads(a.transactions.read_text());meta={r['header']:r for r in map(json.loads,a.meta.read_text().splitlines())};proc=subprocess.Popen([str(a.matcher)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.out.with_suffix('.stderr')).open('w'),text=True)
with a.out.open('w') as f:
 for i in range(1,min(len(tx),a.limit or len(tx))):
  for lag in [1,2]:
   if i<lag:continue
   prev,cur=tx[i-lag],tx[i];source=meta[cur['ref']]['source_path'];target=meta[prev['ref']]['source_path'];seed=np.linalg.inv(prev['seed'])@np.array(cur['seed']);q=dict(case_id=f'{i}_{lag}',source_path=source,target_path=target,target_sha=hashlib.sha256(target.encode()).hexdigest(),seed_common=seed.tolist(),roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0]);tick=time.perf_counter_ns();proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();r=json.loads(proc.stdout.readline());r.update(index=i,lag=lag,dt=(cur['ref']-prev['ref'])*1e-9,header_ns=cur['ref'],pair_wall_ms=(time.perf_counter_ns()-tick)*1e-6);f.write(json.dumps(r)+'\n');f.flush()
proc.stdin.close();proc.wait(timeout=30)
