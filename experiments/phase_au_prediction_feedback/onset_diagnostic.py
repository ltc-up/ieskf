"""Sign-run description on already recovered history; no physical error threshold."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--series',type=Path,required=True);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();z=np.load(a.series);plan=json.loads(a.plan.read_text());rows=[];v=z['balance_prior_difference']
for w in plan['sections']:
 i=w['first_frame'];j=i
 while j>0 and np.sign(v[j-1])==np.sign(v[i]):j-=1
 rows.append(dict(section=w['section'],entry_frame=i,same_sign_run_begin_frame=j,relative_time_s=float(z['t'][j]),begin_difference_m=float(v[j]),interpretation='same-sign prior minus NDT run only, not independently identified physical-error onset'))
a.output.write_text(json.dumps(rows,indent=2)+'\n')
