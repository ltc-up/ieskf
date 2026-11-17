"""Freeze read-only geometric tests before viewing reference scores."""
import sys,json,hashlib
from pathlib import Path
root=Path(sys.argv[1]);old=Path(sys.argv[2]);d=json.load(open(root/'inputs/series.json'));req=json.load(open(old/'phase_bg_far_lio/c4/frozen/requests.json'));plan=[]
plus=next(r for r in req if r['identity']['map_variant']=='1013_plus_tent')
for i in sorted(set(list(range(0,100,10))+[56,57,58,59,60,61,62])):
 r=d['series'][0]['records'][i]
 for variant,path in {**d['global_maps'],'map1013_plus_saved':plus['target_path']}.items():
  plan.append(dict(id=f'causal_{i}_{variant}',t=r['t'],source=r['source'],target=path,seed=r['seed'],crop=True,detail=i in [57,58,59,60],owner_proxy=r['owner_proxy'],solve=False))
# Original historical source/target/seed identities, separately evaluated from rebuilt C5 stream.
for idx,r in enumerate(req):
 tm=r['identity'].get('bag_relative',-1)
 if any(abs(tm-x)<.002 for x in [517.936,517.986,518.136]):
  assert hashlib.sha256(Path(r['target_path']).read_bytes()).hexdigest()==r['target_sha']
  plan.append(dict(id=f'old_{idx}_seed',t=r['header'],source=r['source_path'],target=r['target_path'],seed=r['seed_common'],crop=False,detail=True,solve=True,map_variant=r['identity']['map_variant']))
  if r['identity'].get('old_final') is not None:
   plan.append(dict(id=f'old_{idx}_old_final',t=r['header'],source=r['source_path'],target=r['target_path'],seed=r['identity']['old_final'],crop=False,detail=True,solve=False,map_variant=r['identity']['map_variant']))
(root/'inputs/geometric_diagnostic_plan.json').write_text(json.dumps(plan,indent=2));print('frozen native factor diagnostics',len(plan))
