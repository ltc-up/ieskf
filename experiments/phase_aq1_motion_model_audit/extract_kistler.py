"""Read-only raw scalar Kistler extraction; private values only."""
import argparse,json,sqlite3
from pathlib import Path

def main(a):
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 d=json.loads(a.bindings.read_text());normal=[j for j in d if j['scene']=='normal'];lo=min(min(c['scan_start_ns'] for c in j['clouds']) for j in normal)-400_000_000;hi=lo+21_000_000_000;c=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);tid,typ=c.execute("select id,type from topics where name='/sensor/kistler/measurement'").fetchone();cls=get_message(typ);ev=[]
 for mid,recv,blob in c.execute('select id,timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(tid,lo,hi)):
  m=deserialize_message(blob,cls);ev.append(dict(kind='wheel',header_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,recorder_ns=recv,message_id=mid,valid=True,speed=m.velocity.x,variance=.01,frame=m.header.frame_id))
 assert {e['frame'] for e in ev}=={'kistler'};a.output.write_text(json.dumps(dict(events=ev,variance_provenance='publisher engineering diagonal; not calibrated',status_qualified=False)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
