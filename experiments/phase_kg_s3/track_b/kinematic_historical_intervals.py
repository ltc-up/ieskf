"""Historical lag diagnostics from already committed, immutable motion intervals.

A lag2 result composes two committed intervals with their ORIGINAL availability
contracts. It does not retrospectively refit either interval at a later query,
and is never presented as strict-previous B0 registration.
"""
import argparse,json
from pathlib import Path
import numpy as np
from lossless_archive import iter_jsonl,read_json


def diagnose(interval_rows,cases):
    rows=list(interval_rows);headers=np.array([r['ref_ns'] for r in rows],dtype=np.int64);out=[]
    for case in cases:
        if not len(rows):out.append(dict(case=case,status='NO_INTERVALS'));continue
        i=int(np.argmin(np.abs(headers-int(case['requested_ref_ns']))));lag=int(case['lag'])
        if lag not in (1,2):raise ValueError('HISTORICAL_LAG_MUST_BE_1_OR_2')
        if i+1<lag:out.append(dict(case=case,status='INSUFFICIENT_COMMITTED_PREHISTORY'));continue
        selected=rows[i-lag+1:i+1]
        if any(a['ref_ns']!=b['prev_ref_ns'] for a,b in zip(selected,selected[1:])):raise ValueError('NONCONTIGUOUS_COMMITTED_INTERVALS')
        K=np.eye(4);L=np.eye(4);legacy_valid=True
        for row in selected:
            K=K@np.asarray(row['delta_T'])
            if row['legacy_delta_T'] is None:legacy_valid=False
            else:L=L@np.asarray(row['legacy_delta_T'])
        dt=(selected[-1]['ref_ns']-selected[0]['prev_ref_ns'])*1e-9
        out.append(dict(case=case,status='COMMITTED_HISTORICAL_COMPOSITION',selected_ref_ns=int(headers[i]),offset_from_requested_ns=int(headers[i])-int(case['requested_ref_ns']),prev_ref_ns=selected[0]['prev_ref_ns'],actual_dt_s=dt,lag=lag,kinematic_delta_T=K.tolist(),legacy_delta_T=L.tolist() if legacy_valid else None,kinematic_speed_norm_m_s=float(np.linalg.norm(K[:3,3])/dt),legacy_speed_norm_m_s=None if not legacy_valid else float(np.linalg.norm(L[:3,3])/dt),constituent_availability=[dict(ref_ns=r['ref_ns'],original_available_ns=r['original_available_ns'],query_available_ns=r['query_available_ns']) for r in selected],uses_future_interval_rewrite=False,NDT_algorithm_comparison=False,B0_strict_previous_registration=lag==1))
    return out

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('intervals');a.add_argument('cases');a.add_argument('output');x=a.parse_args()
    rows=diagnose(iter_jsonl(x.intervals),read_json(x.cases));Path(x.output).write_text(json.dumps(rows,indent=2))
