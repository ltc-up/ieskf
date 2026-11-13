"""Canonical time labels and preservation checks; never recomputes any candidate."""
import argparse,copy,hashlib,json
from pathlib import Path

def canonical_case(record, original):
    r=copy.deepcopy(record)
    b=next(b for b in original['branches'] if b['branch']==original['selected'])
    if (r['header_ns']!=original['ref'] or r['prior']!=b['prior'] or
        r['original_ndt']!=b['ndt'] or r['original_update']!=b['update'] or
        r['original_R']!=b['update'].get('R')):
        raise ValueError('FROZEN_ORIGINAL_RECORD_CHANGED')
    old=r.pop('actual_published_state',None)
    branch=r.get('actual_branch_state',old)
    if branch!=b['state']:raise ValueError('BRANCH_SNAPSHOT_CHANGED')
    r.pop('actual_state_stamp',None)
    r.update(actual_branch_state=branch,actual_branch_state_stamp=branch['t'],
             actual_published_base_pose=original['pose'],actual_published_P=original['P'],
             actual_published_state_stamp=original['state_stamp'],
             actual_published_velocity=None,
             actual_published_velocity_status='NOT_EXPORTED_AT_FINAL_STAMP_IN_ORIGINAL_TRACE',
             time_contract='native header / branch snapshot time / final published base pose-P time are distinct')
    return r

def run(root,out):
    if out.exists():raise ValueError('REFUSE_OVERWRITE')
    out.mkdir()
    inputs=[root/'results_be5_stage2a/frozen_54/frames.jsonl',root/'results_be5_stage2a/supplement_1070/frames.jsonl']
    saved={}
    for p in inputs:
        for line in p.open():
            r=json.loads(line);assert r['index'] not in saved;saved[r['index']]=r
    assert len(saved)==49
    canonical=[];mismatch=0
    for line in (root/'results/controlled2_matched_lap/frames.jsonl').open():
        original=json.loads(line);i=original['index']
        if i not in saved:continue
        old=saved[i]
        assert hashlib.sha256(line.encode()).hexdigest()==old['saved_row_sha256']
        r=canonical_case(old,original)
        mismatch+=r['actual_branch_state_stamp']!=r['actual_published_state_stamp']
        assert r['Legacy_candidate']==old['Legacy_candidate']
        assert r['Native_Raw_candidate']==old['Native_Raw_candidate']
        assert r['A_shadow']==old['A_shadow']
        canonical.append(dict(kind='BE',**r))
    for line in (root/'results_be5_stage2a/frozen_54/historical_vertical.jsonl').open():
        canonical.append(dict(kind='historical_Z',**json.loads(line)))
    historic=json.load(open(root/'results_be5_stage2a/frozen_54/historical12632.json'))
    canonical.append(dict(kind='historical12632',**historic))
    assert len(canonical)==55
    with (out/'cases.jsonl').open('x') as f:
        for r in canonical:f.write(json.dumps(r,allow_nan=False)+'\n')
    summary=dict(cases=55,BE_preserved_exactly=49,branch_vs_publication_stamp_mismatch=mismatch,
                 candidate_R_K_delta_P_unchanged=True,original_files_unchanged=True,
                 newly_computed_candidates=0,new_matches=0,new_replay=0,new_state_commits=0,
                 source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 final_published_velocity='UNKNOWN, not replaced by branch snapshot',
                 canonical_file='cases.jsonl')
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.root,a.out)
