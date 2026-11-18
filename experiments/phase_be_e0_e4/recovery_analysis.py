"""Offline closed-loop divergence and bounded-revision accounting, not a pure IMU test."""
import argparse,json,collections
from pathlib import Path
import numpy as np
from evaluate import native,stats
from ndt_global.validation import static_transform
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();c=json.load(open(a.root/'runtime/paths.json'));motion=json.load(open(c['motion']));lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3];out={}
for window in ['lap','multilap']:
 r=a.root/'results'/f'revised_robust_{window}';base=a.root/'results'/f'controlled2_matched_{window}'
 if not (r/'summary.json').exists() or not (base/'summary.json').exists():continue
 rows=[json.loads(s) for s in (r/'frames.jsonl').open()];other=[json.loads(s) for s in (base/'frames.jsonl').open()];posts=[];baseline=[];corrections=[];recovery=[]
 for row,control in zip(rows,other):
  b=next(v for v in row['branches'] if v['branch']==row['selected']);d=next(v for v in control['branches'] if v['branch']==control['selected']);z,R=native(b,lever,row['ref']);posts.append(z);baseline.append(native(d,lever,control['ref'])[0]);prior=np.asarray(b['prior']['p'])-np.asarray(b['prior']['R'])@lever
  if row['index']>=330 and b['update']['accepted']:recovery.append(row['index']);corrections.append(float(np.linalg.norm(z-prior)))
 diff=np.linalg.norm(np.asarray(posts)-np.asarray(baseline),axis=1);summary=json.load(open(r/'summary.json'));ledgers={}
 for b in ['ndt','gicp']:
  ledger=json.load(open(r/(b+'_lifecycle.json')));ledgers[b]=dict(statuses=dict(collections.Counter(v['status'] for v in ledger['records'].values())),revisions=len(ledger['revisions']))
 out[window]=dict(requested_retraction_groups=(len(rows)-1)//200,successful_weight_revisions=len(summary.get('weight_revisions',[])),nonrevocable_groups=summary['decisions'].get('REVOCATION_OUTSIDE_HISTORY',0),branch_ledgers=ledgers,first_selected_accepted_index_after_rejection=recovery[0] if recovery else None,first_reentry_correction_m=corrections[0] if corrections else None,reentry_and_later_correction_m=stats(corrections),divergence_from_matched_at_end_of_interval_m=float(diff[329]),divergence_first_ten_recovery_frames_m=stats(diff[330:340]),whole_loop_divergence_m=stats(diff),interpretation='Different closed-loop matching/latency feedback; not pure prediction drift or certified ground-truth error')
(a.root/'results/recovery_analysis.json').write_text(json.dumps(out,indent=2))
