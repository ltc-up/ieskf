"""Offline short-horizon relative-motion engineering evaluation only.

No fitted alignment, parameter selection or online import. GNSS/INS is an
engineering reference; static gauges cancel in each relative transform.
"""
import argparse
import collections
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from lossless_archive import open_text


def stat(v):
    a=np.asarray(v,float)
    return dict(n=len(a),rmse=float(np.sqrt(np.mean(a*a))),p95_abs=float(np.percentile(abs(a),95)),p99_abs=float(np.percentile(abs(a),99)),max_abs=float(abs(a).max()),mean_signed=float(a.mean())) if len(a) else dict(n=0)


def relative_components(A0,A1,R0,R1,dt):
    if dt<=0:raise ValueError('nonpositive relative interval')
    motion=np.linalg.inv(A0)@A1
    expected=np.linalg.inv(R0)@R1
    error=np.linalg.inv(expected)@motion
    return dict(xy_m=float(np.linalg.norm(error[:2,3])),forward_m=float(error[0,3]),lateral_m=float(error[1,3]),vertical_m=float(error[2,3]),yaw_deg=float(np.rad2deg(np.arctan2(error[1,0],error[0,0]))),translation_rate_error_mps=float(np.linalg.norm(error[:3,3])/dt))


def relative_audit(run, evaluator_file, horizons=(.05,.10,.15,1.,5.), tolerance=.005):
    sys.path.insert(0,str(Path(evaluator_file).resolve().parent))
    spec=importlib.util.spec_from_file_location('kg_s3_offline_evaluator',evaluator_file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    run=Path(run);cfg=json.loads((run/'config.frozen.json').read_text());origin=cfg['start_ns']-130*10**9
    reference=module.EngineeringReference(origin)
    rows=[]
    for line in open_text(run/'frames.jsonl'):
        try:r=json.loads(line)
        except json.JSONDecodeError:break
        if r.get('B'):rows.append(r)
    ts=np.array([r['ref_ns'] for r in rows],dtype=np.int64)
    R={lag:[reference.pose(int(t),lag) for t in ts] for lag in [0.,.075,.15]}
    values=collections.defaultdict(lambda:collections.defaultdict(list));missing=collections.Counter()
    for i,row in enumerate(rows):
        bag_s=(int(ts[i])-origin)*1e-9
        windows=['all']+(['fault515_525'] if 515<=bag_s<=525 else [])+(['pass705_725'] if 705<=bag_s<=725 else [])
        for horizon in horizons:
            goal=int(ts[i])-round(horizon*1e9);k=int(np.searchsorted(ts,goal));candidates=[j for j in [k-1,k] if 0<=j<i]
            if not candidates:missing[str(horizon)]+=1;continue
            j=min(candidates,key=lambda z:abs(int(ts[z])-goal))
            if abs(int(ts[j])-goal)>tolerance*1e9:missing[str(horizon)]+=1;continue
            dt=(int(ts[i])-int(ts[j]))*1e-9
            for mode,x in row['B']['variants'].items():
                before=rows[j]['B']['variants'][mode]
                for lag,poses in R.items():
                    if poses[i] is None or poses[j] is None:continue
                    components=relative_components(np.asarray(before['O_T_base']),np.asarray(x['O_T_base']),poses[j],poses[i],dt)
                    for window in windows:
                        for axis,value in components.items():values[(window,str(horizon),str(lag),mode)][axis].append(value)
    return dict(kind='offline relative engineering discrepancy; not independent ground truth',frames=len(rows),reference=reference.provenance,horizons_s=list(horizons),nearest_anchor_tolerance_s=tolerance,actual_dt_used=True,online_parameter_selection=False,alignment_fit=False,missing_anchor_by_horizon=dict(missing),summary=[dict(window=w,horizon_s=h,lag_s=l,variant=m,metrics={k:stat(v) for k,v in metrics.items()}) for (w,h,l,m),metrics in values.items()])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--shared-evaluator',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    result=relative_audit(a.run,a.shared_evaluator);Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(frames=result['frames'],summary_rows=len(result['summary']),missing=result['missing_anchor_by_horizon'])))
