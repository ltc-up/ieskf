"""Authorized historical inputs only. Full records go to ignored output directory."""
import argparse,json,time,hashlib,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.common import sha,dump,C,CONFIG_PATH
from geometry_ieskf.replay import EventReplay
from ndt_global.shadow import Contract,Candidate,PoseHistory,ShadowCorrector

def read(root,rel):return json.loads((root/rel).read_text())
def candidate(row, frame, contract):
    m=row['measurement'];request=row['request']
    if int(request['header']) != frame['header_ns'] or int(m['header']) != frame['header_ns']:raise ValueError('INPUT_TIME_MISMATCH')
    return Candidate(frame['header_ns'],frame['source_sha256'],frame['source_sha256'],m['final'],contract,
        bool(m['converged']),m.get('fitness'),'OK' if m.get('candidate_valid') else 'INVALID',
        tuple(f'recorded_lidar_{i}' for i,_ in enumerate(frame['source_cloud_identities'])),
        hashlib.sha256(json.dumps(request['seed_common']).encode()).hexdigest(),m.get('error',''))

def main(a):
    root,out,pub=Path(a.data_root),Path(a.private_output),Path(a.public_output)
    out.mkdir(parents=True,exist_ok=True);pub.mkdir(parents=True,exist_ok=True)
    stream='history/phase_v/results/input_continuous_extension.json';contract_path='history/phase_v/results/input_contract.json'
    data=read(root,stream);contract=read(root,contract_path)
    assert sha(CONFIG_PATH)==contract['configuration_sha256'],'FROZEN_CONFIG_REQUIRED'
    ledger=[json.loads(s) for s in (root/'phase_aj_ndt_pose_update/ndt_transactions.jsonl').read_text().splitlines()]
    assert len(ledger)==len(data['frames'])==284
    mapsha=sha(root/'inputs/map_target.bin');assert mapsha==contract['target_sha256']
    meta=Contract(mapsha,hashlib.sha256(json.dumps(contract['extrinsic'],sort_keys=True).encode()).hexdigest(),'AL-local-epoch1')
    h=PoseHistory(meta);shadow=ShadowCorrector(h);current=[None];idx=[0];private=[];wire=[];times=[]
    source_hashes=[]
    def load(e):
        current[0]=e;p=root/'history/phase_v'/e['source_file'];assert sha(p)==e['source_sha256'];source_hashes.append(e['source_sha256']);return np.load(p)
    control=EventReplay(contract,geometry=False,source_loader=load).run(data,None)
    class Bypass:
        def correct(self,x,P,source,unused):
            i=idx[0];e=current[0];row=ledger[i];T=replay.factor.transform(x)
            h.add(x.t,T,meta);c=candidate(row,e,meta);t=time.perf_counter_ns();event=shadow.consume(c,x.t);p=shadow.publish(x.t);times.append((time.perf_counter_ns()-t)/1e6)
            private.append(dict(candidate=c.__dict__|{'contract':meta.__dict__},local_pose=T.tolist(),event=event,publication=p))
            wire.extend([dict(schema_version=1,kind='odom',stamp_ns=x.t,pose=T.tolist(),contract=meta.__dict__),dict(schema_version=1,kind='candidate',candidate=c.__dict__|{'contract':meta.__dict__},now_ns=x.t)])
            idx[0]+=1
            return x.copy(),P.copy(),dict(accepted=False,reason='MAP_ODOM_SHADOW_ONLY',wall_ms=0.)
    replay=EventReplay(contract,geometry=True,source_loader=load);replay.solver=Bypass();result=replay.run(data,None)
    fields=['p_imu_map','v_imu_map','q_map_body_xyzw','bg_body_rad_s','ba_body_m_s2','P']
    for arow,brow in zip(control['outputs'],result['outputs']):
        for k in fields:np.testing.assert_array_equal(arow[k],brow[k])
    assert len(control['outputs'])==len(result['outputs'])==1470
    lastgood=max(i for i,r in enumerate(private) if r['event']['accepted'])
    c=candidate(ledger[lastgood],data['frames'][lastgood],meta)
    duplicate=shadow.consume(c,c.stamp_ns)['reason']
    from dataclasses import replace
    delayed=shadow.consume(replace(c,correlation_group='delay'),c.stamp_ns+600_000_000)['reason']
    bad=np.array(c.pose);bad[2,3]+=100
    before=[T.copy() for T in h.poses]
    injected=shadow.consume(replace(c,pose=bad,correlation_group='injected'),c.stamp_ns)
    for A,B in zip(before,h.poses):np.testing.assert_array_equal(A,B)
    residuals=[]
    for r in private:
        if r['publication']['published'] and r['event']['accepted']:
            residuals.append(float(np.max(abs(np.array(r['publication']['pose'])-np.array(r['candidate']['pose'])))))
    reasons=collections.Counter(r['event']['reason'] for r in private)
    counts=dict(reasons);public=dict(qualification='PARTIAL',real_frames=284,duration_s=result['elapsed_sensor_s'],local_output_count=1470,
       local_source='fresh replay of Bosch IMU + historical scalar wheel channel; historical fused initialization only',
       source_hashes_checked=len(set(source_hashes)),local_full_state_P_bitwise_equal=True,shadow_decisions=counts,
       trusted_anchors=0,published_shadow=sum(r['publication']['published'] for r in private),
       composition_max_abs=max(residuals),duplicate_reason=duplicate,delayed_reason=delayed,
       injected_wrong_candidate_accepted_shadow=injected['accepted'],local_history_unchanged=True,
       core_ms={'p50':float(np.median(times)),'p95':float(np.percentile(times,95)),'max':max(times)},
       statistical_calibration=False,causal_frontend='NOT_REGENERATED; recorded AJ seeds and frozen historical Deskew are correlated inputs',
       input_stream_sha256=sha(root/stream),target_sha256=mapsha,configuration_sha256=sha(CONFIG_PATH),
       ledger_sha256=sha(root/'phase_aj_ndt_pose_update/ndt_transactions.jsonl'))
    dump(out/'local_control.json',control);dump(out/'local_shadow.json',result);dump(out/'decisions.json',private)
    (out/'ros_events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in wire))
    dump(pub/'results.json',public);print(json.dumps(public,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--private-output',required=True);p.add_argument('--public-output',required=True);main(p.parse_args())
