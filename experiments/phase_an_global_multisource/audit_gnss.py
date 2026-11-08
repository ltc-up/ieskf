"""Read-only ROS2 bag extraction. Never emits coordinates to public output."""
import argparse,json,sqlite3,hashlib,collections
from pathlib import Path
import numpy as np

def main(a):
    from rclpy.serialization import deserialize_message
    from vectornav_msgs.msg import GpsGroup
    root,out,pub=Path(a.data_root),Path(a.private_output),Path(a.public_output);out.mkdir(parents=True,exist_ok=True)
    d=json.loads((root/'history/phase_v/results/input_continuous_extension.json').read_text());lo,hi=d['begin_ns'],d['end_ns']
    con=sqlite3.connect(f'file:{Path(a.bag).resolve()}?mode=ro&immutable=1',uri=True)
    topics={name:(tid,typ) for tid,name,typ in con.execute('select id,name,type from topics')}
    records=[];digests=[]
    for index,topic in enumerate(['/vectornav/raw/gps','/vectornav/raw/gps2']):
        tid,typ=topics[topic];assert typ=='vectornav_msgs/msg/GpsGroup'
        for mid,received,blob in con.execute('select id,timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo-1_000_000_000,hi+1_000_000_000)):
            m=deserialize_message(blob,GpsGroup);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            if not lo<=stamp<=hi:continue
            digest=hashlib.sha256(blob).hexdigest();digests.append(digest)
            records.append(dict(source=f'gnss{index+1}',stamp_ns=stamp,recorder_ns=received,message_id=mid,cdr_sha256=digest,
              frame_id=m.header.frame_id,group_fields=int(m.group_fields),fix=int(m.fix),numsats=int(m.numsats),
              poslla=[m.poslla.x,m.poslla.y,m.poslla.z],posu=[m.posu.x,m.posu.y,m.posu.z],
              position_field_enabled=bool(m.group_fields & m.GPSGROUP_POSLLA),fix_field_enabled=bool(m.group_fields & m.GPSGROUP_FIX),uncertainty_enabled=bool(m.group_fields & m.GPSGROUP_POSU),
              timeinfo_status=int(m.timeinfo_status),qualified=False,
              qualification_failures=['MAP_DATUM_UNVERIFIED','ANTENNA_LEVER_REFERENCE_UNVERIFIED','HARDWARE_HEADER_ALIGNMENT_UNVERIFIED','UNCERTAINTY_STATISTICS_UNVERIFIED','CROSS_SOURCE_CORRELATION_UNKNOWN']))
    records.sort(key=lambda e:(e['stamp_ns'],e['source']));con.close()
    bysource={s:sum(e['source']==s for e in records) for s in ['gnss1','gnss2']}
    receipt=[(e['recorder_ns']-e['stamp_ns'])/1e6 for e in records]
    health={s:{'nonmonotonic':sum(y['stamp_ns']<=x['stamp_ns'] for x,y in zip([e for e in records if e['source']==s],[e for e in records if e['source']==s][1:])),'max_gap_ms':float(max(np.diff([e['stamp_ns'] for e in records if e['source']==s]),default=0)/1e6)} for s in bysource}
    summary=dict(real_messages=len(records),counts=bysource,fix_histogram=dict(collections.Counter(e['fix'] for e in records)),
      enabled_position=sum(e['position_field_enabled'] for e in records),enabled_uncertainty=sum(e['uncertainty_enabled'] for e in records),
      finite_position=sum(np.isfinite(e['poslla']).all() for e in records),all_positive_posu=sum(all(v>0 for v in e['posu']) for e in records),
      recorder_minus_header_ms={'min':min(receipt,default=0),'p50':float(np.median(receipt)) if receipt else None,'max':max(receipt,default=0)},
      temporal_health=health,qualified_global_observations=0,rtk_fixed_evidence='NOT_ESTABLISHED_FROM_THIS_MESSAGE_SCHEMA',
      cdr_group_sha256=hashlib.sha256(''.join(sorted(digests)).encode()).hexdigest(),
      failure_reasons=dict(collections.Counter(reason for e in records for reason in e['qualification_failures'])),
      coordinates_published=False)
    (out/'gnss_records.json').write_text(json.dumps(records,indent=2)+'\n');(pub/'gnss_audit.json').write_text(json.dumps(summary,indent=2,default=lambda x:int(x))+'\n');print(json.dumps(summary,indent=2,default=lambda x:int(x)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--bag',required=True);p.add_argument('--private-output',required=True);p.add_argument('--public-output',required=True);main(p.parse_args())
