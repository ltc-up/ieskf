"""Resolve frozen public window indices against authorized native headers."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,required=True);p.add_argument('--transactions',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads(a.plan.read_text());tx=json.loads(a.transactions.read_text());rows=[]
for w in plan['sections']:
 row=dict(w);row['analysis_begin_ns']=int(tx[w['first_frame']]['ref']-round(w['padding_s']*1e9));row['analysis_end_ns']=int(tx[w['last_frame']]['ref']+round(w['padding_s']*1e9));rows.append(row)
if a.output.exists():assert json.loads(a.output.read_text())==rows,'FROZEN_WINDOW_MISMATCH'
else:a.output.write_text(json.dumps(rows))
