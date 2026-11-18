"""Read-only independent checks on a complete-line prefix of common-runner output."""
import argparse
import collections
import json
from pathlib import Path
import numpy as np
from lossless_archive import open_text,read_json


def audit(path,capture=None,archive_manifest=None):
    rows=[];partial=0;decoded_bytes=0
    actual_path=Path(path)
    if not actual_path.exists():actual_path=actual_path.with_name(actual_path.name+'.gz')
    with open_text(path) as f:
        for line in f:
            decoded_bytes+=len(line.encode('utf-8'))
            try:r=json.loads(line)
            except json.JSONDecodeError:partial+=1;break
            if r.get('B'):rows.append(r)
    if not rows:return dict(complete_rows=0,partial_tail=partial)
    violations=[];modes=rows[0]['B']['variants'];stats={}
    has_pure='B-P' in modes
    O0=np.array(rows[0]['B']['variants']['B-P' if has_pure else 'B0']['O_T_base']);U0=np.array(rows[0]['independent_U']);H=O0@np.linalg.inv(U0)
    pure_errors=[];source_per_branch=0
    for i,r in enumerate(rows):
        group=r['B']
        if has_pure:pure_errors.append(float(np.max(np.abs(np.array(group['variants']['B-P']['O_T_base'])-H@np.array(r['independent_U'])))))
        for mode,z in group['variants'].items():
            if z.get('source_identity') and z['source_identity']!=group['source_identity']:violations.append([i,mode,'different Source hash'])
            source_per_branch+=bool(z.get('source_identity'))
            if mode=='B-P' and z.get('target_points',0):violations.append([i,mode,'pure-motion Target built'])
            if i==0:continue
            before=rows[i-1];prev=before['B']['variants'][mode]
            pred=np.array(prev['O_T_base'])@np.linalg.inv(np.array(before['independent_U']))@np.array(r['independent_U'])
            if not np.allclose(z['predicted_O_T_base'],pred,atol=1e-8):violations.append([i,mode,'prediction is not previous O times delta U'])
            if not z['accepted'] and not np.allclose(z['O_T_base'],pred,atol=1e-8):violations.append([i,mode,'rejection advanced proposal'])
            for h in z.get('targets',[]):
                if h['age_ns']<=0:violations.append([i,mode,'nonpast Target'])
                if 'selected_point_ids' in h:violations.append([i,mode,'unexpected ID list'])
            if mode=='B0' and z.get('targets'):
                h=z['targets'][0]
                if h['frame_id']!=before['B']['frame_id']:violations.append([i,mode,'Target is not previous actual frame'])
                if h['predicted_anchor']!=(not prev.get('accepted',False)):violations.append([i,mode,'bad failed-anchor flag'])
                if not np.allclose(h['O_T_base'],prev['O_T_base'],atol=1e-8):violations.append([i,mode,'Target anchor pose changed'])
            val=(z.get('proposal') or {}).get('validation')
            if val and not np.isclose(val['actual_dt_s'],(r['ref_ns']-before['ref_ns'])*1e-9,rtol=0,atol=1e-12):violations.append([i,mode,'nominal rather than actual dt'])
    for mode in modes:
        zz=[r['B']['variants'][mode] for r in rows];times=np.array([z['full_step_ms'] for z in zz])
        reasons=collections.Counter(reason for z in zz for reason in ((z.get('proposal') or {}).get('validation') or {}).get('reasons',[]))
        stats[mode]=dict(status=dict(collections.Counter(z['status'] for z in zz)),step_ms=dict(p50=float(np.median(times)),p95=float(np.percentile(times,95)),p99=float(np.percentile(times,99)),max=float(times.max())),rejection_reasons=dict(reasons))
    capture_result=None
    if capture:
        expected=json.loads(Path(capture).read_text());matches=[r for r in rows if r['source_sha256']==expected['source_file_sha256']]
        capture_result=dict(matches=len(matches),payload_size_matches=expected['payload_size_matches'])
        if matches:
            r=matches[0];capture_result.update(source_payload_hash_matches=r['B']['source_identity']==expected['source_payload_sha256'],dense_xyz_hash_matches=r['B']['dense_identity']==expected['dense_xyz_sha256'],source_n_matches=r['source_n']==expected['source_n'])
    byte_count=decoded_bytes if actual_path.suffix=='.gz' else actual_path.stat().st_size
    byte_source='decompressed UTF-8 bytes read (no archive manifest supplied)' if actual_path.suffix=='.gz' else 'original uncompressed file stat'
    if archive_manifest is not None:
        entries=read_json(archive_manifest)['files']
        candidates=[v for v in entries if Path(v['archive_file']).name==actual_path.name]
        if len(candidates)!=1:raise ValueError('ARCHIVE_MANIFEST_ENTRY_NOT_UNIQUE')
        expected_bytes=int(candidates[0]['original_bytes'])
        if expected_bytes!=byte_count:raise ValueError('ARCHIVE_MANIFEST_ORIGINAL_BYTE_COUNT_MISMATCH')
        byte_source='archive manifest original_bytes, cross-checked against decompressed stream'
        byte_count=expected_bytes
    return dict(kind='independent read-only causal prefix audit; not final trajectory accuracy',complete_rows=len(rows),partial_tail=partial,first_ref_ns=rows[0]['ref_ns'],last_ref_ns=rows[-1]['ref_ns'],source_per_branch_checks=source_per_branch,max_pure_U_chain_element_error=max(pure_errors) if pure_errors else None,initial_O_preserves_U_tilt=bool(np.allclose(O0[:3,:3][2],U0[:3,:3][2],atol=1e-8)),violations=violations,variants=stats,source_capture=capture_result,frames_bytes=byte_count,frames_bytes_source=byte_source,stored_artifact_bytes=actual_path.stat().st_size,projected_frames_bytes_at_7900=int(byte_count/max(1,len(rows))*7900),live_100Hz_validated=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('frames');p.add_argument('--capture');p.add_argument('--output');p.add_argument('--archive-manifest');a=p.parse_args();r=audit(a.frames,a.capture,a.archive_manifest);s=json.dumps(r,indent=2)
    if a.output:Path(a.output).write_text(s+'\n')
    print(s)
