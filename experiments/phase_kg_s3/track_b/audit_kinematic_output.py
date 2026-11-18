"""Read-only causal and chain checks; neither accuracy nor real-time certification."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from lossless_archive import open_text


def rows(path):
    with open_text(path) as f:
        for line in f:
            try:yield json.loads(line)
            except json.JSONDecodeError:break  # A live prefix may end mid-write.


def audit(events, frames):
    originals={str(r['B']['frame_id']):r for r in frames if r.get('B')}
    max_original_ref=max((r['ref_ns'] for r in originals.values()),default=-1);deferred=[]
    errors=[];counts=Counter();reasons=Counter();previous=None;H=None;max_chain=0.;segments=0;delayed=set();committed=0
    def require(condition,frame,rule):
        if not condition:errors.append(dict(frame_id=frame,rule=rule))
    for e in events:
        counts[e['status']]+=1;reasons[e.get('reason','')]+=1
        if e['status'] not in ('DELAY','COMMITTED'):continue
        fid=str(e['frame_id']);original=originals.get(fid)
        if original is None and e['ref_ns']>max_original_ref:deferred.append(fid)
        else:require(original is not None,fid,'original frame available')
        if original is not None:
            require(e['source_identity']==original['source_payload_sha256'],fid,'same immutable current Source')
            require(e['original_cloud_available_ns']==original['available_ns'],fid,'original cloud available preserved')
            require(e['ref_ns']==original['ref_ns'],fid,'original Deskew reference preserved')
        require(e['query_available_ns']>=e['original_cloud_available_ns'],fid,'no query before release')
        require(e['delay_ns']==e['query_available_ns']-e['original_cloud_available_ns'],fid,'actual delay')
        if e['status']=='DELAY':
            require(e.get('delta_T') is None,fid,'delay has no fabricated motion');delayed.add(fid);continue
        committed+=1;interval=e['interval'];d=np.asarray(interval['delta_T']);u=np.asarray(e['U_kin'])
        require(interval['status']=='VALID',fid,'only valid interval committed')
        require(interval['query_available_ns']==e['query_available_ns'],fid,'actual retry availability')
        require(interval['bias_base_rad_s']==[0.,0.,0.],fid,'fixed zero gyro bias')
        require(interval['body_lateral_vertical_zero'] is True,fid,'explicit nonholonomic assumption')
        cursor=interval['prev_ref_ns']
        for s in interval['segments']:
            segments+=1
            require(s['start_ns']==cursor and s['end_ns']>cursor,fid,'contiguous positive integration segments')
            for kind,limit in [('imu',25_000_000),('wheel',100_000_000)]:
                require(s[kind+'_header_ns']<=s['start_ns'],fid,kind+' no future header')
                require(s[kind+'_available_ns']<=e['query_available_ns'],fid,kind+' no future available')
                require(s['end_ns']-s[kind+'_header_ns']<=limit,fid,kind+' held limit')
            cursor=s['end_ns']
        require(cursor==e['ref_ns'],fid,'integration covers reference')
        if previous is None:
            H=np.asarray(e['H_init']);require(H.shape==(4,4),fid,'fixed H_init initialized')
            require(np.allclose(u,d,rtol=0,atol=1e-10),fid,'initial U is first physical interval')
        else:
            require(e['H_init'] is None,fid,'H_init never reset')
            require(interval['prev_ref_ns']==previous['ref_ns'],fid,'strict adjacent interval')
            err=float(np.max(np.abs(np.asarray(previous['U_kin'])@d-u)));max_chain=max(max_chain,err)
            require(err<1e-9,fid,'Ukin only integrates physical delta')
        for label,z in e['variants'].items():
            target=z['target'];require(label=='KINEMATIC_BK__'+target,fid,'two-axis label')
            require(z['predictor']=='KINEMATIC_BK',fid,'predictor label')
            require(z.get('source_identity',e['source_identity'])==e['source_identity'],fid,'same Source across targets')
            if previous is None:continue
            p=previous['variants'][label];pred=np.asarray(p['O_T_base'])@d
            require(np.allclose(z['predicted_O_T_base'],pred,rtol=0,atol=1e-8),fid,label+' previous O times physical delta')
            if not z['accepted']:require(np.allclose(z['O_T_base'],pred,rtol=0,atol=1e-8),fid,label+' rejected proposal not fed back')
            for h in z.get('targets',[]):
                require(h['age_ns']>0,fid,'past-only historical Target')
                if 'roi_center_O' in h:
                    require(np.allclose(h['roi_center_O'],pred[:3,3],rtol=0,atol=1e-8),fid,'physical ROI center is causal O prediction')
                    require(h['roi_radius']==120 and h['roi_relative_z']==[-10,30] and h['voxel_leaf']==.8,fid,'frozen historical Target crop and sampling')
            if target=='PREVIOUS' and z.get('targets'):
                hs=z['targets'];require(len(hs)==1 and hs[0]['frame_id']==previous['frame_id'],fid,'strict previous real scan')
                require(np.allclose(hs[0]['O_T_base'],p['O_T_base'],rtol=0,atol=1e-8),fid,'stored historical anchor unchanged')
            val=(z.get('proposal') or {}).get('validation')
            if val:require(abs(val['actual_dt_s']-(e['ref_ns']-previous['ref_ns'])*1e-9)<1e-12,fid,'actual scan dt used')
        previous=e
    return dict(qualification='independent causal output audit; not accuracy, statistical P, or 100Hz certification',committed=committed,deferred_source_joins_buffered_prefix=deferred,event_counts=dict(counts),reasons=dict(reasons),delayed_frames=len(delayed),checked_segments=segments,max_Ukin_chain_element_error=max_chain,violations=errors)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--output');a=p.parse_args();root=Path(a.run)
    result=audit(rows(root/'kinematic_events.jsonl'),rows(root/'frames.jsonl'));s=json.dumps(result,indent=2)
    if a.output:Path(a.output).write_text(s+'\n')
    print(s)
