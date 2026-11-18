"""Descriptive utilization from saved scalar candidates; no candidate updates."""
import json,sys
from pathlib import Path
import numpy as np

def main(out):
 out=Path(out);rows=json.load(open(out/'scalar_candidates.json'));result={}
 for g in ['normal8','right3','straight_control_first20_corner0_from100','largest10_translation_innovation_no_reference_selection','BE_saved']:
  result[g]={}
  for m in [5,2,1]:
   z=[r for r in rows if g in r['groups'] and r['multiplier']==m];v=[r['reference']['0.075'] for r in z if r['reference']['0.075']['reference_valid']];good=[r for r in z if r['reference']['0.075'].get('ndt_xy',999)<=.5];rat=[]
   for r in z:
    e=r['reference']['0.075']
    if e['reference_valid'] and abs(e['ndt_lateral']-e['prior_lateral'])>=.05:rat.append(r['delta_lateral']/(e['ndt_lateral']-e['prior_lateral']))
   result[g][str(m)]={'all_valid_xy_improved_count':sum(r['candidate_xy']<r['prior_xy'] for r in v),'all_valid_count':len(v),'proxy_good_count':len(good),'proxy_good_would_admit':sum(r['would_admit'] for r in good),'proxy_good_xy_improved_count':sum(r['reference']['0.075']['candidate_xy']<r['reference']['0.075']['prior_xy'] for r in good),'lateral_fraction_for_abs_innovation_ge_0_05m_count':len(rat),'median_signed_lateral_fraction':float(np.median(rat)) if rat else None}
 (out/'utilization_aggregate.json').write_text(json.dumps(result,indent=2))
if __name__=='__main__':main(*sys.argv[1:])
