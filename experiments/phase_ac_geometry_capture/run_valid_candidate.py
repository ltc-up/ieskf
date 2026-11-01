"""AC2/3/4 predetermined frozen runs, no continuous prediction."""
import argparse,json,time,numpy as np
from pathlib import Path
from compare_states import load_first
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.state import inject
from geometry_ieskf.objective import state
from geometry_ieskf.capture import AlternatingCapture,starting_seeds
from geometry_ieskf.qualification import pair_retention
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence

p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);aa,data,contract,e,frame,source,prior,P,factor,original_query=load_first(D);query=NearestValidCorrespondence(original_query.target)
assert not (O/'seed_results.json').exists(),'Do not accidentally repeat the frozen capture experiment'
seeds=starting_seeds(P);dump(O/'predetermined_seeds.json',seeds);runner=AlternatingCapture(factor,max_outer=60);results=[]
def stream(name):
 def write(row):
  with open(O/(name+'.iterations.jsonl'),'a') as f:f.write(json.dumps(row,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v.item())+'\n')
 return write
for seed in seeds:
 runner.on_iteration=stream(seed['id'])
 result=runner.run(prior,P,source,query,seed['z']);result['seed_id']=seed['id'];dump(O/(seed['id']+'.json'),result);results.append(result)
 print(seed['id'],result['status'],result['outer_iterations'],result['final']['gn_white_norm'],result['final']['newton_decrement'],result['final_original_guard_KKT']['active'],result['wall_ms'],flush=True)
dump(O/'seed_results.json',results)
reps=[];frames={f['header_ns']:f for f in data['frames']}
for ordinal in [1,3,215]:
 event=aa['lidar_updates'][ordinal];x=state(event['predicted']);Q=np.array(event['predicted']['P']);srcpath=D/'history/phase_v'/frames[event['header_ns']]['source_file'];assert sha(srcpath)==frames[event['header_ns']]['source_sha256'];src=np.load(srcpath);runner.on_iteration=stream('representative_'+str(ordinal));r=runner.run(x,Q,src,query);r['ordinal']=ordinal;reps.append(r)
 print('representative',ordinal,r['status'],r['outer_iterations'],r['final']['gn_white_norm'],r['final_original_guard_KKT']['active'],r['wall_ms'],flush=True)
dump(O/'representative_results.json',reps)
# Endpoint witnesses use the original numerical tolerance, never a fitted basin radius.
L=np.linalg.cholesky(P);comparisons=[]
for i,r in enumerate(results):
 for j,s in enumerate(results[:i]):
  cr=query.get(inject(prior,np.array(r['z'])),source,factor);cs=query.get(inject(prior,np.array(s['z'])),source,factor);distance=float(np.linalg.norm(np.linalg.solve(L,np.array(r['z'])-np.array(s['z']))))
  comparisons.append({'seed_a':r['seed_id'],'seed_b':s['seed_id'],'prior_white_distance':distance,'endpoint_pairs':pair_retention(cr,cs),'both_interior_qualified':r['interior_qualified'] and s['interior_qualified'],'same_numerical_endpoint':distance<=C['convergence_whitened_step'] and pair_retention(cr,cs)['identical']})
dump(O/'endpoint_comparisons.json',comparisons)
