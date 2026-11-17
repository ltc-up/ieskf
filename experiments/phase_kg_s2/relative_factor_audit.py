"""Read-only per-point audit of actual optimized binary factors, no re-solve."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);cfg=json.load(open(root/'inputs/series.json'));S={s['id']:s for s in cfg['series']};plan=[]
for label,indices in [('fault',[57,58,59,60]),('pass713',[15,16,17,18,20])]:
 rec={r['t']:r for r in S[label]['records']};folder=root/'native_verified'/f'{label}_E2';steps=list(map(json.loads,open(folder/'steps.jsonl')));groups={steps[i]['t']:{}for i in indices}
 for q in map(json.loads,open(folder/'retrospective.jsonl')):
  if q['decision_t']in groups:groups[q['decision_t']][q['node']]=q
 gf={r['t']:r for r in map(json.loads,open(folder/'geometry.jsonl'))}
 for i in indices:
  row=steps[i];t=row['t'];states=groups[t]
  def T(st):
   A=np.eye(4);A[:3,:3]=np.array(st['T_map_base_R']);A[:3,3]=np.array(st['p_base']).ravel();return A
  for j,f in enumerate(gf[t]['factors']):
   assert len(f['keys'])==2
   targetnode=f['keys'][0]&((1<<56)-1);sourcenode=f['keys'][1]&((1<<56)-1);a=states[targetnode];b=states[sourcenode];rel=np.linalg.inv(T(a['state']))@T(b['state']);targettime=a['state_t'];plan.append(dict(id=f'{label}_E2_step{i}_edge{j}',source=rec[t]['source'],target=rec[targettime]['source'],seed=rel.tolist(),t=t,target_t=targettime,dt_ms=(t-targettime)*1e-6,detail=True,crop=False,solve=False,qualification='actual binary factor endpoint diagnostic in fixed target-base chart; not independent registration, target is unchanged historical source'))
(root/'inputs/relative_posthoc_plan.json').write_text(json.dumps(plan,indent=2));print(len(plan),'actual optimized relative edge audits')
