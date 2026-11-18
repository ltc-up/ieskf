"""Read-only full input identity audit for the independent 713 s replay.

Keep the legal 130/135 s history. End at the final complete three-LiDAR
release no later than 725 s, avoiding a silently discarded partial triplet.
This script does not run an estimator, choose algorithm parameters, or read
navigation references.
"""
import argparse, collections, json, sqlite3, sys
from pathlib import Path
from types import SimpleNamespace

REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/'experiments/phase_bg_far_lio/c1'))
import continuous_inputs as ci
from provenance import require_run_output


def main(config_path, output):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    cfg=json.loads(Path(config_path).read_text());output=Path(output)
    require_run_output(output);output.mkdir(exist_ok=False)
    upper=ci.ORIGIN+725_000_000_000
    db=sqlite3.connect('file:'+cfg['bag']+'?mode=ro&immutable=1',uri=True)
    topics={i:n for i,n in db.execute('select id,name from topics')}
    tids=[i for i,n in topics.items() if n in ci.NAMES]
    query='select id,topic_id,timestamp,data from messages where timestamp>=? and timestamp<=? and topic_id in ('+','.join('?' for _ in tids)+') order by timestamp,id'
    pending={n:collections.deque() for n in ci.NAMES};last=None
    for ident,tid,available,blob in db.execute(query,[upper-2_000_000_000,upper,*tids]):
        msg=deserialize_message(blob,PointCloud2)
        pending[topics[tid]].append(dict(id=ident,available=available,header=ci.ns(msg.header.stamp)))
        if all(pending.values()):
            group=[pending[n].popleft() for n in ci.NAMES]
            if max(g['header'] for g in group)-min(g['header'] for g in group)>10_000_000:
                # Start of the small boundary query may bisect one old triplet.
                pending={n:collections.deque() for n in ci.NAMES};continue
            last=group
    db.close()
    if last is None:raise RuntimeError('NO_COMPLETE_TRIPLET_NEAR_725')
    end=max(x['available'] for x in last)
    ci.START=cfg['start_ns'];ci.INIT=cfg['init_ns'];ci.END=end
    boundary=dict(start_ns=ci.START,init_ns=ci.INIT,end_ns=end,requested_upper_ns=upper,last_complete_triplet=last,reason='FULL_LEGAL_PREFIX_FOR_713',reference_used=False)
    (output/'boundary.json').write_text(json.dumps(boundary,indent=2))
    ci.audit_stream(SimpleNamespace(root=str(output),bag=cfg['bag'],audit=cfg['audit'],extrinsics=cfg['extrinsics']))
    proof=json.loads((output/'timing_contract/summary.json').read_text())
    if proof['reason']!='COMPLETED' or any(proof['tail']['pending_tail'].values()):
        raise RuntimeError('EXTENDED_INPUT_CONTRACT_FAILED')
    for name in ['A0_B_all_0105_v2','A0_B_all_1013_v2','A1_0105','A1_1013','A2_0105_approved','A2_1013_approved']:
        source=Path(config_path).parent/(name+'.json');candidate=json.loads(source.read_text())
        candidate['end_ns']=end;candidate['expected_ordered_identity_sha256']=proof['ordered_identity_sha256']
        candidate['output']=candidate['output']+'_pass713'
        candidate['extended_input_proof']=str(output/'timing_contract/summary.json')
        candidate['evaluation_scope']='independent full 130 s input / 135 s initialization through complete triplet near 725 s'
        (output/(name+'_pass713.json')).write_text(json.dumps(candidate,indent=2))
    print(json.dumps(boundary),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config');p.add_argument('output');a=p.parse_args();main(a.config,a.output)
