"""Independent relative-motion evaluation of every committed local chain.

Short-baseline and accumulated-horizon errors are gauge invariant. Predictions
and rejected proposals are compared against the previous COMMITTED pose, never
against a previous proposal. No result is read by online code.
"""
import argparse,bisect,collections,json
from pathlib import Path
import numpy as np
from evaluate import EngineeringReference,errors,stats
from evaluate_kinematic import finite_pose,window_names
from log_io import open_text
from provenance import require_run_output


def relative_error(previous,current,reference_previous,reference_current):
    return errors(np.linalg.inv(previous)@current,np.linalg.inv(reference_previous)@reference_current)


def evaluate(run):
    run=Path(run);out=run/'kinematic_relative_evaluation';require_run_output(out);out.mkdir(exist_ok=False)
    cfg=json.loads((run/'config.frozen.json').read_text());origin=cfg['start_ns']-130*10**9
    ref=EngineeringReference(origin);series=collections.defaultdict(dict);metadata={}
    for filename in ('frames.jsonl','kinematic_events.jsonl'):
        for line in open_text(run/filename):
            row=json.loads(line)
            if filename=='frames.jsonl':
                if 'ndt' not in row:continue
                variants=(row.get('B') or {}).get('variants',{})
            else:
                if row['status']!='COMMITTED':continue
                variants=row['variants']
            t=int(row['ref_ns'])
            for name,x in variants.items():
                if t in series[name]:raise ValueError('DUPLICATE_VARIANT_REFERENCE')
                values={}
                for label,key in [('final','O_T_base'),('prediction','predicted_O_T_base')]:
                    if key in x:values[label]=finite_pose(x[key])
                if (x.get('proposal') or {}).get('O_T_base') is not None:values['proposal']=finite_pose(x['proposal']['O_T_base'])
                series[name][t]=values;metadata[(name,t)]=x['status']
    result=collections.defaultdict(lambda:collections.defaultdict(list));invalid=collections.Counter();counts=collections.Counter()
    with (out/'errors.jsonl').open('x') as log:
        for name,poses in series.items():
            stamps=sorted(poses)
            for i,t in enumerate(stamps):
                if not i:continue
                for horizon in ('previous',.5,1.):
                    j=i-1 if horizon=='previous' else bisect.bisect_right(stamps,t-int(horizon*1e9))-1
                    if j<0:continue
                    a=stamps[j]
                    if horizon!='previous' and abs((t-a)*1e-9-horizon)>.075:continue
                    previous=poses[a].get('final')
                    if previous is None:invalid[name+'/anchor']+=1;continue
                    for lag in (0.,.075,.15):
                        RA=ref.pose(a,lag);RB=ref.pose(t,lag)
                        if RA is None or RB is None:continue
                        for label,T in poses[t].items():
                            if T is None:invalid[name+'/'+label]+=1;continue
                            e=relative_error(previous,T,RA,RB);dt=(t-a)*1e-9
                            record=dict(variant=name,label=label,horizon=str(horizon),ref_ns=t,previous_ref_ns=a,dt_s=dt,lag_s=lag,status=metadata[name,t],errors=e)
                            if lag==.075 and horizon=='previous':log.write(json.dumps(record)+'\n')
                            counts[name+'/'+label+'/'+str(horizon)]+=1
                            for w in window_names((t-origin)*1e-9):
                                for axis,v in e.items():result[(w,str(lag),name,label,str(horizon))][axis].append(v)
    summary=dict(reference=ref.provenance,scope='relative local motion; every committed frame retained, no fitted gauge',horizons='actual strict previous / last source at or before .5s and 1s, tolerance .075s',invalid=dict(invalid),counts=dict(counts),summary=[dict(window=w,lag=lag,variant=name,label=label,horizon=h,metrics={a:stats(v) for a,v in m.items()}) for (w,lag,name,label,h),m in result.items()])
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(dict(variants=len(series),invalid=dict(invalid))))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');evaluate(p.parse_args().run)
