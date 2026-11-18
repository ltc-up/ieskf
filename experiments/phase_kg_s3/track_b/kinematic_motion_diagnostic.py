"""CPU-only original-ledger motion replay; no clouds, GPU or reference reader.

This diagnoses input coverage and raw-vs-posterior motion increments only. It is
not a local NDT trajectory or independent accuracy assessment.
"""
import argparse,hashlib,json,sqlite3,sys,time
from pathlib import Path
import numpy as np
from kinematic_predictor import HeaderCoverageAudit,IntervalKinematicPredictor
from lossless_archive import read_json


def run(run_path,header_metadata_path,out_path):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TwistWithCovarianceStamped
    run=Path(run_path);out=Path(out_path);out.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((run/'config.frozen.json').read_text());meta=read_json(header_metadata_path)
    h={k:[r['header_ns'] for r in meta['records'] if r['kind']==k] for k in ('imu','wheel')}
    audit=HeaderCoverageAudit(h['imu'],h['wheel'],meta['covers_from_ns'],meta['covers_to_ns'],meta['identity'])
    p=IntervalKinematicPredictor(cfg['init_ns'],audit);R=np.asarray(json.loads(Path(cfg['extrinsics']).read_text())['T_base_imu'])[:3,:3]
    refs={}
    for line in (run/'frames.jsonl').open():
        row=json.loads(line)
        if row.get('B'):refs[row['ref_ns']]=(row['available_ns'],np.asarray(row['independent_U']))
    db=sqlite3.connect('file:'+str(cfg['bag'])+'?mode=ro&immutable=1',uri=True)
    prev=cfg['init_ns'];previous_U=None;pending=[];committed=0;delays=0;counts={};speeds=[];legacy_speeds=[];max_delay=0;ordered=hashlib.sha256();started=time.perf_counter();terminal=None
    with (out/'intervals.private.jsonl').open('x') as f:
        for line in (run/'input_ledger.jsonl').open():
            e=json.loads(line);a=e['available_ns'];kind=e['kind'];ordered.update(json.dumps(e['identities'],sort_keys=True).encode());counts[kind]=counts.get(kind,0)+1
            if kind in ('imu','wheel'):
                ident=e['identities'][0];raw=db.execute('SELECT data,timestamp FROM messages WHERE id=?',(ident['id'],)).fetchone()
                if raw is None or raw[1]!=a or hashlib.sha256(raw[0]).hexdigest()!=ident['cdr_sha256']:raise ValueError('RAW_LEDGER_IDENTITY_MISMATCH')
                m=deserialize_message(raw[0],Imu if kind=='imu' else TwistWithCovarianceStamped);header=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
                if header!=e['header_ns']:raise ValueError('RAW_HEADER_MISMATCH')
                event=dict(kind=kind,header_ns=header,available_ns=a,message_id=ident['id'])
                if kind=='imu':event['omega_base']=(R@np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])).tolist()
                else:event.update(speed_base=m.twist.twist.linear.x,variance=m.twist.covariance[0])
                p.ingest(event)
                if committed==0 and not pending and header<cfg['init_ns']:p.prune_before(cfg['init_ns'])
            elif kind=='cloud' and e['header_ns'] in refs:pending.append(e['header_ns'])
            while pending:
                ref=pending[0];original,U=refs[ref];result=p.query(prev,ref,a,original)
                if result['status']=='DELAY':delays+=1;break
                if result['status']!='VALID':terminal=result;break
                dt=(ref-prev)*1e-9;delta=np.asarray(result['delta_T']);speed=float(np.linalg.norm(delta[:3,3])/dt);speeds.append(speed)
                old=None if previous_U is None else np.linalg.inv(previous_U)@U
                if old is not None:legacy_speeds.append(float(np.linalg.norm(old[:3,3])/dt))
                max_delay=max(max_delay,a-original)
                f.write(json.dumps(dict(ref_ns=ref,prev_ref_ns=prev,original_available_ns=original,query_available_ns=a,delta_T=delta.tolist(),legacy_delta_T=None if old is None else old.tolist(),speed_norm_m_s=speed,interval_provenance=result))+'\n')
                pending.pop(0);prev=ref;previous_U=U;committed+=1;p.prune_before(prev)
            if terminal:break
    db.close()
    def stats(x):return dict(n=len(x),p50=float(np.median(x)),p99=float(np.percentile(x,99)),max=float(max(x))) if x else dict(n=0)
    summary=dict(kind='CPU raw motion input/increment diagnostic only',reference_reader=False,GPU=False,header_audit_identity=meta['identity'],ordered_input_identity=ordered.hexdigest(),input_counts=counts,expected_legal_refs=len(refs),committed=committed,delay_attempts=delays,maximum_delay_ns=max_delay,last_committed_ref_ns=prev,pending=pending,terminal=terminal,kinematic_increment_speed_norm_m_s=stats(speeds),legacy_posterior_difference_speed_norm_m_s=stats(legacy_speeds),wall_s=time.perf_counter()-started)
    (out/'summary.private.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('run');a.add_argument('header_metadata');a.add_argument('output');x=a.parse_args();run(x.run,x.header_metadata,x.output)
