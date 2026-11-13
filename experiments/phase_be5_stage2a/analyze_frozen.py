"""Read the already selected <=60 cases; no new frames or estimator commits."""
import argparse,json,collections,csv
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State,inject
from geometry_ieskf.vertical_velocity import fixed_normal_model,body_normal_model

def stats(v):
    a=np.asarray(v,float)
    return dict(n=len(a),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),max=float(np.max(a))) if len(a) else dict(n=0)
def run(folder,out,supplement=None):
    if out.exists():raise ValueError('REFUSE_OVERWRITE')
    out.mkdir()
    rows=[json.loads(l) for l in (folder/'frames.jsonl').open()]
    old=[json.loads(l) for l in (folder/'historical_vertical.jsonl').open()]
    if supplement is not None:
        rows.extend(json.loads(l) for l in (supplement/'frames.jsonl').open())
    assert len({r['index'] for r in rows})==len(rows)
    assert len(rows)+len(old)+1<=60
    summary=dict(frames=len(rows)+len(old)+1,BE_frames=len(rows),historical_Z_frames=len(old),
                 all_actual_stage2a_commits=0,independent_vertical_truth=False,
                 C_fusion_blocked=len(rows)+len(old)+1,Native_real_frame_scope='saved BE numerical map only; no deployed ROS/old-Z equivalence')
    q=json.load(open(folder/'summary.json'));summary['frozen_algebra_ms']=q['diagnostic_ms']
    summary['legacy_delta_proposal_max_parity_error']=q['legacy_delta_proposal_max_parity_error']
    summary['fitness_range']=[min(r['original_ndt']['fitness'] for r in rows),max(r['original_ndt']['fitness'] for r in rows)]
    summary['pose_modes']={}
    for mode in ('Legacy_candidate','Native_Raw_candidate'):
        selected=[r[mode] for r in rows if 'delta_proposed' in r[mode]]
        summary['pose_modes'][mode]=dict(n=len(selected),would_admit=sum(z['original_update_would_admit'] for z in selected),
          abs_dz=stats([abs(z['delta_proposed'][2]) for z in selected]),abs_dvz=stats([abs(z['delta_proposed'][5]) for z in selected]),
          R_eigen_min=stats([np.linalg.eigvalsh(z['R'])[0] for z in selected]),
          R_eigen_max=stats([np.linalg.eigvalsh(z['R'])[-1] for z in selected]),
          max_abs_vz_contribution_sum_error=max(abs(sum(z['vz_signed_innovation_contributions'])-z['delta_proposed'][5]) for z in selected))
    summary['A']={}
    for label in ('A1_fixed_map_up_hypothesis','A2_body_up_hypothesis'):
        a=[r['A_shadow'][label] for r in rows+old if label in r['A_shadow']]
        summary['A'][label]=dict(n=len(a),physical_fusion_eligible=0,original_domain_exceeded=sum(bool(z['original_domain_failures']) for z in a),
              abs_h=stats([abs(z['prediction']) for z in a]),abs_dz=stats([abs(z['delta_proposed'][2]) for z in a]),
              abs_dvz=stats([abs(z['delta_proposed'][5]) for z in a]),
              P_change_Frobenius=stats([np.linalg.norm(z['P_change']) for z in a]))
    # Six-dimensional innovation contribution row already exactly follows K*r.
    # Record all six signed components privately, not in public trajectories.
    detail=[]
    for r in rows:
        for mode in ('Legacy_candidate','Native_Raw_candidate'):
            z=r[mode]
            if 'delta_proposed' not in z:continue
            detail.append([r['case_id'],mode,*z['z_signed_innovation_contributions'],
                           *z['vz_signed_innovation_contributions'],z['delta_proposed'][2],z['delta_proposed'][5]])
    with (out/'signed_gain_contributions.csv').open('x') as f:
        w=csv.writer(f);w.writerow(['private_case_id','mode',*[f'dz_from_{i}' for i in range(6)],*[f'dvz_from_{i}' for i in range(6)],'dz_sum','dvz_sum']);w.writerows(detail)
    # Segment labels are frozen offline selection metadata, not online evidence.
    groupnames=('fixed_segment_3','fixed_segment_9','fixed_segment_5','fixed_segment_8',
                'high_fitness_in_this_baseline','large_saved_vz_proposal','offline_proxy_bad_not_truth','supplement_offline_raw_match_proxy_bad_not_truth')
    summary['groups']={}
    for g in groupnames:
        a=[r for r in rows if g in r['selection_reasons']]
        item=dict(frames=len(a),interpretation='overlapping selection groups; no independent correctness labels')
        for mode in ('Legacy_candidate','Native_Raw_candidate'):
            candidates=[r[mode] for r in a if 'delta_proposed' in r[mode]]
            item[mode]=dict(would_admit=sum(z['original_update_would_admit'] for z in candidates),
                           abs_dvz=stats([abs(z['delta_proposed'][5]) for z in candidates]))
        summary['groups'][g]=item
    summary['historical_candidate_vz']=[dict(window=r['case']['window'],
      historical_V_returned=[g['delta_vz_map'] for g in r['historical_signed_vz_decomposition']['groups'] if g['label']=='V_returned'],
      proposed_Y=[dict(label=g['label'],dvz=g['delta_vz_map']) for g in r['historical_signed_vz_decomposition']['groups'] if g['label'] in ('Y_fixed_constrained','Y_dynamic_last_constrained')])
      for r in old]
    (out/'aggregate.json').write_text(json.dumps(summary,indent=2))
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--supplement',type=Path);a=p.parse_args()
    s=run(a.input,a.out,a.supplement);print(json.dumps(s,indent=2))
