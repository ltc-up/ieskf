"""Read-only real-ledger v1/v2 snapshot-query equivalence; no GPU/reference."""
import argparse,hashlib,json,sqlite3,time
from pathlib import Path
import numpy as np
from lossless_archive import iter_jsonl
from kinematic_realtime_support import MotionStore as Original
from kinematic_realtime_support_v2 import MotionStore as Cached


def validate(run):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TwistWithCovarianceStamped
    root=Path(run);cfg=json.loads((root/'config.frozen.json').read_text());R=np.asarray(json.loads(Path(cfg['extrinsics']).read_text())['T_base_imu'])[:3,:3]
    original=Original(cfg['init_ns']);cached=Cached(cfg['init_ns']);expected={e['ref_ns']:e for e in iter_jsonl(root/'kinematic_events.jsonl') if e['status']=='COMMITTED'}
    db=sqlite3.connect('file:'+cfg['bag']+'?mode=ro&immutable=1',uri=True);previous=cfg['init_ns'];n=0;motions=0;differences=[];times={'original_snapshot_ms':[],'cached_snapshot_ms':[]};begin=time.monotonic()
    for e in iter_jsonl(root/'input_ledger.jsonl'):
        if e['kind'] in ('imu','wheel'):
            ident=e['identities'][0];raw=db.execute('SELECT data,timestamp FROM messages WHERE id=?',(ident['id'],)).fetchone()
            if raw[1]!=e['available_ns'] or hashlib.sha256(raw[0]).hexdigest()!=ident['cdr_sha256']:raise ValueError('RAW_LEDGER_IDENTITY_MISMATCH')
            m=deserialize_message(raw[0],Imu if e['kind']=='imu' else TwistWithCovarianceStamped);header=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            if header!=e['header_ns']:raise ValueError('RAW_HEADER_MISMATCH')
            event=dict(kind=e['kind'],header=header,available=e['available_ns'],id=ident['id'])
            if e['kind']=='imu':event['omega']=(R@np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])).tolist()
            else:event.update(speed=m.twist.twist.linear.x,variance=m.twist.covariance[0])
            original.append(event);cached.append(event);motions+=1
            if n==0 and header<cfg['init_ns']:original.prune(cfg['init_ns']);cached.prune(cfg['init_ns'])
        elif e['kind']=='cloud' and e['header_ns'] in expected:
            ref=e['header_ns'];results=[]
            for name,store in [('original_snapshot_ms',original),('cached_snapshot_ms',cached)]:
                t=time.perf_counter_ns();p=store.predictor();times[name].append((time.perf_counter_ns()-t)/1e6);results.append(p.query(previous,ref,e['available_ns'],e['available_ns']))
            if results[0]!=results[1]:differences.append(dict(ref_ns=ref,kind='complete query output differs'))
            if results[0]['delta_T']!=expected[ref]['interval']['delta_T']:differences.append(dict(ref_ns=ref,kind='frozen continuous delta differs'))
            if results[0]['status']!='VALID':raise ValueError('NOT_VALID_REAL_INTERVAL')
            n+=1;previous=ref;original.prune(ref);cached.prune(ref)
    db.close()
    def stat(a):return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(max(a)))
    return dict(scope='all real motion queries exact snapshot-cache equivalence; no reference/GPU',committed_intervals=n,expected_intervals=len(expected),raw_CDR_identity_checked_motion_events=motions,differences=differences,snapshot_times={k:stat(v) for k,v in times.items()},wall_s=time.monotonic()-begin)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--output',required=True);a=p.parse_args();r=validate(a.run);Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
