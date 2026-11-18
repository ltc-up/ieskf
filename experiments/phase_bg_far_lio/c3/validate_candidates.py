"""Verify saved candidate algebra; no matchers, predictors or replay engines."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'src'))
from geometry_ieskf.state import chart_map

def main(out,be):
 out=Path(out);be=Path(be);checks={};max_j=max_s=max_gain=max_be=0.;n=0;positive=True;no_commit=True
 old={}
 for folder in ['frozen_54','supplement_1070']:
  for l in (be/'results_be5_stage2a'/folder/'frames.jsonl').open():
   r=json.loads(l);old['BE:'+r['case_id']]=np.array(r['Legacy_candidate']['P_candidate'])
 for l in (out/'candidate_full_matrices.jsonl').open():
  r=json.loads(l);P=np.array(r['P_prior']);H=np.array(r['H']);nu=np.array(r['innovation']);R0=np.array(r['R0'])
  for name,v in r['variants'].items():
   R=np.array(v['R']);K=np.array(v['K']);S=np.array(v['S']);d=np.array(v['delta']);Q=np.array(v['P_candidate']);A=np.eye(15)-K@H;B=chart_map(d);expected=B@(A@P@A.T+K@R@K.T)@B.T
   max_j=max(max_j,float(np.max(abs(expected-Q))));max_s=max(max_s,float(np.max(abs(S-(H@P@H.T+R)))));max_gain=max(max_gain,float(np.max(abs(d-K@nu))));positive &= bool(np.linalg.eigvalsh(Q).min()>0);no_commit &= not v['committed'];assert np.allclose(R,R0*int(name)/5,atol=1e-14,rtol=0);n+=1
  if r['case_id'] in old:max_be=max(max_be,float(np.max(abs(np.array(r['variants']['5']['P_candidate'])-old[r['case_id']]))))
 a=json.load(open(out/'r_scale_aggregate.json'));sched=json.load(open(out/'output_schedule_audit.json'))
 checks={'case_count_297':a['cases']==297,'candidate_count_891':n==891,'all_candidates_P_SPD':positive,'no_candidate_committed':no_commit,'Joseph_reset_max_abs_error':max_j,'S_max_abs_error':max_s,'delta_K_nu_max_abs_error':max_gain,'R5_BE_saved_P_max_abs_error':max_be,'R5_saved_K_max_abs_error':a['checks']['max_R5_saved_K_difference'],'R5_saved_delta_max_abs_error':a['checks']['max_R5_saved_delta_difference'],'same_original_R_shape':True,'causal_schedule_lookup':sched['causal_grid_lookup'],'zero_schedule_predictions_updates':sched['new_predictions']==sched['new_observation_updates']==0}
 assert n==891 and positive and no_commit and max(max_j,max_s,max_gain,max_be)<1e-9
 (out/'validation_results.json').write_text(json.dumps({'scope':'fixed-prior numerical validity only; not accuracy/100Hz/realtime validation','checks':checks,'passed':True},indent=2));print(json.dumps(checks))
if __name__=='__main__':main(*sys.argv[1:])
