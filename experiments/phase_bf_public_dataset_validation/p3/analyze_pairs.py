"""Read completed limited ON/OFF measurements; never starts matching."""
import argparse,csv,json
from pathlib import Path
import numpy as np

def stats(v):return dict(n=len(v),min=float(min(v)),median=float(np.median(v)),max=float(max(v)),mean=float(np.mean(v)))
def analyze(path):
 s=json.loads(Path(path).read_text());assert s['calls']==s['completed']==18
 records=s['records'];rows=[];pairs=[]
 for r in records:
  z=r['result'];assert all(v['equal'] for v in r['historical_equivalence'].values())
  assert (z['fitness'] is None)==(r['mode']=='OFF') and not z['legacy_R_generated']
  additive=z['p2_prepare_ms']+z['p3_pre_sync_ms']+z['align_wall_ms']+z['p3_post_align_sync_ms']+z['quality_extra_ms']+z['p3_post_quality_sync_ms']
  rest=r['service_ms']-additive;assert rest>=0
  rows.append(dict(frame=r['frame'],repeat=r['repeat'],mode=r['mode'],align_ms=z['align_wall_ms'],pcl_ms=z['p2_pcl_fit_ms'],pre_sync_ms=z['p3_pre_sync_ms'],post_align_sync_ms=z['p3_post_align_sync_ms'],quality_extra_ms=z['quality_extra_ms'],post_quality_sync_ms=z['p3_post_quality_sync_ms'],prepare_ms=z['p2_prepare_ms'],service_ms=r['service_ms'],ipc_serialization_residual_ms=rest,fitness_available=z['fitness_available'],target_build_cached_ms=z['target_build_ms']))
 for f in [60,65,76]:
  for rep in range(3):
   on=next(x for x in rows if x['frame']==f and x['repeat']==rep and x['mode']=='ON');off=next(x for x in rows if x['frame']==f and x['repeat']==rep and x['mode']=='OFF')
   delta=on['align_ms']-off['align_ms'];tail=lambda x:x['post_align_sync_ms']+x['quality_extra_ms']+x['post_quality_sync_ms']
   pairs.append(dict(frame=f,repeat=rep,align_saved_ms=delta,on_pcl_ms=on['pcl_ms'],saving_minus_pcl_ms=delta-on['pcl_ms'],off_minus_on_post_align_and_quality_ms=tail(off)-tail(on),inclusive_align_through_quality_saved_ms=delta+tail(on)-tail(off)))
 summary=[]
 for f in [60,65,76]:
  result=dict(frame=f)
  for name,predicate in [('all3',lambda r:True),('steady_rounds1_2',lambda r:r['repeat']>0)]:
   v=[x for x in rows if x['frame']==f and predicate(x)];ps=[x for x in pairs if x['frame']==f and predicate(x)]
   result[name]=dict(ON_align=stats([x['align_ms'] for x in v if x['mode']=='ON']),OFF_align=stats([x['align_ms'] for x in v if x['mode']=='OFF']),align_saving=stats([x['align_saved_ms'] for x in ps]),ON_pcl=stats([x['on_pcl_ms'] for x in ps]),inclusive_saving=stats([x['inclusive_align_through_quality_saved_ms'] for x in ps]),OFF_minus_ON_tail=stats([x['off_minus_on_post_align_and_quality_ms'] for x in ps]))
  summary.append(result)
 a=dict(baseline='0ed77cb8f80a5a170eb5dac644ed6df5abbf6712',calls=18,frames=[60,65,76],repeats_per_mode=3,all_equivalent=True,maximum_historical_numeric_difference=max(v.get('max_abs',0) for r in records for v in r['historical_equivalence'].values()),fitness_off_is_missing=True,legacy_R_created=False,filter_updates=0,replays=0,trace_enabled=False,
  max_post_align_sync_OFF_ms=max(x['post_align_sync_ms'] for x in rows if x['mode']=='OFF'),maximum_absolute_tail_change_ms=max(abs(x['off_minus_on_post_align_and_quality_ms']) for x in pairs),summary=summary,pairs=pairs,gpu_kernel_event_time='UNKNOWN_NOT_EXPOSED_PRIVATE_STREAM',qualification='NOT_20HZ_OR_PRODUCTION_QUALIFICATION')
 return rows,a

def main(a):
 out=Path(a.out);out.mkdir(parents=True,exist_ok=False);rows,s=analyze(a.receipt)
 (out/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
 with (out/'timings.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0],lineterminator='\n');w.writeheader();w.writerows(rows)
 print(json.dumps(s,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--receipt',required=True);p.add_argument('--out',required=True);main(p.parse_args())
