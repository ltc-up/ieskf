"""Independent streaming audit of actual geometric replay invocations."""
from log_io import open_text
import argparse,hashlib,json,struct
from collections import Counter
from pathlib import Path
import numpy as np

def audit(root):
 latest={};counts=Counter();issues=[];transitions=[];max_revision=0;white_geo=[];white_prior=[];white_ratios=[];white_cos=[];rawprior_P_changes=[]
 for line in open_text(Path(root)/'event_lifecycle.jsonl'):
  e=json.loads(line);i=e['observation_id'];u=e['info'];p=e['prior'];q=e['posterior'];counts['semantic_revisions']+=1;max_revision=max(max_revision,e['revision'])
  vals=np.r_[p['p'],p['v'],np.asarray(p['R']).ravel(),p['bg'],p['ba'],np.asarray(p['P']).ravel()]
  h=hashlib.sha256(vals.astype('<f8').tobytes()+struct.pack('<q',int(p['t']))).hexdigest()
  if u.get('prior_hash') is not None and u['prior_hash']!=h:issues.append(dict(id=i,revision=e['revision'],reason='PRIOR_HASH_MISMATCH'))
  white=None;rawchange=None;m=u.get('selected_metrics',{})
  if 'prior_gradient' in m and 'geometry_gradient' in m:
   gp=np.asarray(m['prior_gradient']);gg=np.asarray(m['geometry_gradient']);P=np.asarray(p['P']);a=float(np.sqrt(max(0,gp@P@gp)));b=float(np.sqrt(max(0,gg@P@gg)));white_geo.append(b);white_prior.append(a);white=dict(geometry=b,prior=a,ratio=b/a if a>0 else None,cosine=float(np.clip(gg@P@gp/(a*b),-1,1)) if a*b>0 else None)
   if a>0:white_ratios.append(b/a)
   else:counts['zero_prior_whitened_gradient_revisions']+=1
   if a*b>0:white_cos.append(float(np.clip(gg@P@gp/(a*b),-1,1)))
  if u.get('selected')=='prior':
   counts['raw_prior_selected_revisions']+=1
   if not all(np.array_equal(np.asarray(p[k]),np.asarray(q[k])) for k in ('p','v','R','bg','ba')):issues.append(dict(id=i,revision=e['revision'],reason='RAW_PRIOR_CHANGED_MEAN'))
   dP=float(np.linalg.norm(np.asarray(q['P'])-np.asarray(p['P'])));rawprior_P_changes.append(dP);rawchange=dP;counts['raw_prior_selected_covariance_changed_revisions']+=dP>0
  yes=bool(u.get('accepted'));counts['accepted_revisions' if yes else 'rejected_revisions']+=1
  if i in latest and latest[i]['accepted'] and not yes:transitions.append(dict(id=i,revision=e['revision'],previous_reason=latest[i]['reason'],new_reason=u.get('reason')))
  if u.get('reason')=='NUMERICAL_DOMAIN_INVALID':
   if not all(np.array_equal(np.asarray(p[k]),np.asarray(q[k])) for k in ('p','v','R','bg','ba','P')):issues.append(dict(id=i,revision=e['revision'],reason='DOMAIN_FAILURE_CHANGED_PREDICTION'))
  Q=np.asarray(q['P']);ev=np.linalg.eigvalsh((Q+Q.T)/2)
  if not np.isfinite(Q).all() or ev.min()<-1e-9 or abs(Q-Q.T).max()>1e-9:issues.append(dict(id=i,revision=e['revision'],reason='INVALID_POSTERIOR_FULLP'))
  latest[i]=dict(accepted=yes,reason=u.get('reason'),revision=e['revision'],white=white,raw_prior_P_change=rawchange)
 reasons=Counter(v['reason'] for v in latest.values())
 def stats(x):
  return None if not x else dict(n=len(x),p50=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(max(x)),min=float(min(x)))
 final_white={k:stats([v['white'][k] for v in latest.values() if v['white'] is not None and v['white'][k] is not None]) for k in ('geometry','prior','ratio','cosine')}
 return dict(final_observation_whitened_gradients=final_white,final_raw_prior_selected_P_change_frobenius=stats([v['raw_prior_P_change'] for v in latest.values() if v['raw_prior_P_change'] is not None]),whitened_metric='sqrt(g.T @ actual_frozen_full_prior_P @ g); diagnostic only, not statistical calibration',whitened_geometry_gradient=stats(white_geo),whitened_prior_gradient=stats(white_prior),whitened_geometry_to_nonzero_prior_ratio=stats(white_ratios),whitened_gradient_cosine=stats(white_cos),raw_prior_selected_P_change_frobenius=stats(rawprior_P_changes),invocation_semantics='semantic revisions; cache-hit equivalent calls may be omitted by sink',counts=dict(counts),distinct_observations=len(latest),maximum_revision=max_revision,final_numerical_accepted=sum(v['accepted'] for v in latest.values()),final_numerical_reason_counts=dict(reasons),accepted_to_rejected_transitions=transitions,issues=issues,reference_used=False,covariance_statistically_calibrated=False)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('--out');a=p.parse_args();s=json.dumps(audit(a.root),indent=2)
 if a.out:Path(a.out).write_text(s+'\n')
 print(s)
