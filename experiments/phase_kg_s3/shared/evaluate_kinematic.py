"""Independent offline KINEMATIC_BK evaluation; no online reader imports it.

Join delayed commits by their ORIGINAL scan reference, never by the frame that
happened to release a retry. Keep prediction, rejected proposal and committed
pose separate. Engineering references are not independent ground truth.
"""
import argparse, collections, json
from pathlib import Path
import numpy as np
from evaluate import EngineeringReference, errors, stats
from provenance import require_run_output
from log_io import open_text

BB_SECONDS=(326.806250994,324.906239426,314.355307261,333.056278567,
            340.555487392,278.306203640,278.956299119,328.556273013)


def window_names(t):
    result=['all']
    for name,lo,hi in [('normal8',307.30626695,313.70623555000003),('right3',339.406271902,343.80606308),
                       ('fault515_525',515.,525.),('pass705_725',705.,725.)]:
        if lo<=t<=hi:result.append(name)
    result.extend('BB_'+str(i+1) for i,x in enumerate(BB_SECONDS) if abs(t-x)<=.25)
    return result


def finite_pose(value):
    a=np.asarray(value,float)
    if a.shape!=(4,4) or not np.isfinite(a).all():return None
    if not np.allclose(a[3],[0,0,0,1],atol=1e-6,rtol=0):return None
    if not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-5,rtol=0):return None
    if abs(np.linalg.det(a[:3,:3])-1)>1e-5:return None
    return a


def evaluate(run):
    run=Path(run);out=run/'kinematic_evaluation';require_run_output(out);out.mkdir(exist_ok=False)
    cfg=json.loads((run/'config.frozen.json').read_text());origin=cfg['start_ns']-130*10**9
    reference=EngineeringReference(origin)
    rows={}
    for line in open_text(run/'frames.jsonl'):
        r=json.loads(line)
        if 'ndt' in r:rows[int(r['ref_ns'])]=r
    commits={};events=collections.Counter();terminal=[];identity_errors=[];delays=[]
    for line in open_text(run/'kinematic_events.jsonl'):
        e=json.loads(line);events[e['status']]+=1
        if e['status']=='COMMITTED':
            t=int(e['ref_ns'])
            if t in commits:raise ValueError('DUPLICATE_KINEMATIC_COMMIT')
            if t not in rows:raise ValueError('KINEMATIC_COMMIT_WITHOUT_SOURCE_FRAME')
            commits[t]=e;delays.append(e['query_available_ns']-e['original_cloud_available_ns'])
            if e['source_identity']!=rows[t]['source_payload_sha256']:identity_errors.append(t)
            if e['query_available_ns']<e['original_cloud_available_ns']:raise ValueError('RETRY_BACKDATED')
        elif e['status']!='DELAY':terminal.append(e)
    gauges={};values=collections.defaultdict(lambda:collections.defaultdict(list))
    status=collections.defaultdict(collections.Counter);nonfinite=collections.Counter()
    previous_ref=None;previous_U=None
    details=(out/'errors.jsonl').open('x')
    for t,row in sorted(rows.items()):
        variants=dict((row.get('B') or {}).get('variants',{}));kin=commits.get(t)
        if kin:variants.update(kin['variants'])
        poses={};relative={};refs={lag:reference.pose(t,lag) for lag in (0.,.075,.15)}
        for name,x in variants.items():
            status[name][x['status']]+=1
            if name not in gauges:
                # A kinematic gauge must be explicit: its kernel U is not map.
                gauge=x.get('T_map_from_O')
                if gauge is None and kin and name in kin['variants']:gauge=kin.get('T_map_from_O')
                if gauge is None and name.startswith('KINEMATIC_BK'):
                    raise ValueError('MISSING_EXPLICIT_KINEMATIC_MAP_GAUGE')
                if gauge is None:
                    if x['status']!='INITIALIZED':raise ValueError('MISSING_LEGACY_INITIAL_GAUGE')
                    gauge=np.linalg.inv(np.asarray(x['O_T_U']))
                gauges[name]=np.asarray(gauge)
            G=gauges[name]
            for label,key in [('final','O_T_base'),('prediction','predicted_O_T_base')]:
                if key in x:
                    T=finite_pose(x[key])
                    if T is None:nonfinite[name+'/'+label]+=1
                    else:poses[name+'/'+label]=G@T
            proposal=x.get('proposal') or {}
            if 'O_T_base' in proposal:
                T=finite_pose(proposal['O_T_base'])
                if T is None:nonfinite[name+'/proposal']+=1
                else:poses[name+'/proposal']=G@T
        if previous_ref is not None:
            dt=(t-previous_ref)*1e-9
            if 'independent_U' in row and previous_U is not None:
                relative['LEGACY_15D_DIFF']=np.linalg.inv(previous_U)@np.asarray(row['independent_U'])
            if kin and int(kin['interval']['prev_ref_ns'])==previous_ref:
                relative['KINEMATIC_BK']=np.asarray(kin['interval']['delta_T'])
            for lag,B in refs.items():
                A=reference.pose(previous_ref,lag)
                if A is None or B is None:continue
                expected=np.linalg.inv(A)@B
                for name,delta in relative.items():
                    e=errors(delta,expected);e['translation_rate_error_mps']=float(np.linalg.norm((np.linalg.inv(expected)@delta)[:3,3])/dt)
                    for w in window_names((t-origin)*1e-9):
                        for k,v in e.items():values[(w,str(lag),'relative_predictor/'+name)][k].append(v)
        record=dict(ref_ns=t,bag_s=(t-origin)*1e-9,by_lag={})
        for lag,B in refs.items():
            if B is None:continue
            record['by_lag'][str(lag)]={}
            for name,T in poses.items():
                e=errors(T,B);record['by_lag'][str(lag)][name]=e
                for w in window_names(record['bag_s']):
                    for k,v in e.items():values[(w,str(lag),name)][k].append(v)
        details.write(json.dumps(record)+'\n')
        previous_ref=t;previous_U=None if 'independent_U' not in row else np.asarray(row['independent_U'])
    details.close()
    missing_refs=sorted(set(rows)-set(commits))
    summary=dict(kind='independent offline prediction/proposal/final engineering evaluation',reference=reference.provenance,
        original_source_frames=len(rows),kinematic_committed_frames=len(commits),frame_coverage_fraction=len(commits)/len(rows) if rows else 0,
        first_committed_ref_ns=min(commits,default=None),last_committed_ref_ns=max(commits,default=None),
        remaining_uncommitted_source_range_ns=None if not missing_refs else [min(missing_refs),max(missing_refs)],
        requested_raw_input_end_ns=cfg['end_ns'],
        events=dict(events),terminal=terminal,source_identity_mismatches=identity_errors,
        real_input_retry_delay_ms=stats(np.asarray(delays)*1e-6),nonfinite_proposals_or_outputs=dict(nonfinite),
        variant_status={k:dict(v) for k,v in status.items()},reference_selection_online=False,alignment_fit=False,
        BB_selection='historical sensor-optical disagreement cases; not correct-peak labels',
        summary=[dict(window=w,lag=l,output=name,metrics={k:stats(v) for k,v in m.items()}) for (w,l,name),m in values.items()])
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:summary[k] for k in ['original_source_frames','kinematic_committed_frames','frame_coverage_fraction','events','source_identity_mismatches']}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');evaluate(p.parse_args().run)
