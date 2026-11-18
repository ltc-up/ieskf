"""Posthoc actual historical NDT support/error summaries; no threshold tuning.

valid_pairs/source is a Gaussian-pair multiplicity, NOT distinct coverage.
Surface diagnostics absent from frozen logs remain NOT_LOGGED, never zero.
"""
import argparse,json,sys
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
from audit_kinematic_output import rows


def stat(a):
    a=np.asarray(a,float)
    return dict(n=len(a),mean=float(a.mean()),p10=float(np.percentile(a,10)),p50=float(np.median(a)),p90=float(np.percentile(a,90)),max=float(a.max()),min=float(a.min()),rmse=float(np.sqrt(np.mean(a*a)))) if len(a) else dict(n=0)


def summarize(run,shared):
    sys.path.insert(0,str(shared));from evaluate import EngineeringReference,errors
    root=Path(run);cfg=json.loads((root/'config.frozen.json').read_text());origin=cfg['start_ns']-130*10**9;reference=EngineeringReference(origin)
    windows=[('all',135.,(cfg['end_ns']-origin)*1e-9),('first5s',135.,140.),('normal8',307.306,313.706),('right3',339.406,343.806),('fault515_525',515.,525.),('pass705_725',705.,725.)]
    values=defaultdict(lambda:defaultdict(list));counts=defaultdict(Counter);previous=None
    for e in rows(root/'kinematic_events.jsonl'):
        if e['status']!='COMMITTED':continue
        seconds=(e['ref_ns']-origin)*1e-9;active=[name for name,a,b in windows if a<=seconds<=b]
        expected={}
        if previous is not None:
            for lag in [0.,.075,.15]:
                a=reference.pose(previous['ref_ns'],lag);b=reference.pose(e['ref_ns'],lag)
                if a is not None and b is not None:expected[str(lag)]=np.linalg.inv(a)@b
        for name,z in e['variants'].items():
            proposal=z.get('proposal') or {};relative={}
            if previous is not None:
                anchor=np.asarray(previous['variants'][name]['O_T_base']);actual=np.linalg.inv(anchor)@np.asarray(z['O_T_base'])
                for lag,truth in expected.items():
                    relative[lag]=dict(final=errors(actual,truth),prediction=errors(np.asarray(e['interval']['delta_T']),truth))
                    if 'O_T_base' in proposal:relative[lag]['proposal']=errors(np.linalg.inv(anchor)@np.asarray(proposal['O_T_base']),truth)
            for window in active:
                key=(window,name);counts[key]['frames']+=1;counts[key][z['status']]+=1
                if z['status']!='INITIALIZED':values[key]['target_points'].append(z['target_points'])
                if proposal:
                    counts[key]['proposal_frames']+=1
                    for field in ['source_points','target_valid_voxels','valid_pairs','neighbor_points','inliers','target_build_ms']:
                        if field in proposal:values[key][field].append(proposal[field])
                    source=proposal.get('source_points',0)
                    if source:
                        for field in ['valid_pairs','neighbor_points','inliers']:
                            if field in proposal:values[key][field+'_per_source'].append(proposal[field]/source)
                    if not any('surface' in k.lower() for k in proposal):counts[key]['surface_NOT_LOGGED']+=1
                else:counts[key]['no_proposal']+=1
                for lag,outputs in relative.items():
                    for label,err in outputs.items():
                        for axis,value in err.items():values[key]['adjacent_'+lag+'/'+label+'/'+axis].append(value)
        previous=e
    return dict(qualification='posthoc support and conditional engineering relative errors, no online use',map=cfg['map'],pair_ratio_definition='valid Gaussian point-pairs / source points; may exceed one, not coverage',distinct_ratio_definition='neighbor_points / source_points',surface='NOT_LOGGED in frozen continuous runtime; helper/test existence does not establish real-data validation',reference=reference.provenance,summary=[dict(window=w,variant=n,counts=dict(counts[(w,n)]),metrics={k:stat(a) for k,a in m.items()}) for (w,n),m in values.items()])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--shared',required=True);p.add_argument('--output',required=True);a=p.parse_args();result=summarize(a.run,a.shared);Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(dict(map=result['map'],summaries=len(result['summary']),surface=result['surface'])))
