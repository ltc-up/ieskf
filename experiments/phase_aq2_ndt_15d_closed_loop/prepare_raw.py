"""Recover immutable raw cloud payloads by exact bag message ID and CDR digest."""
import argparse,json,sqlite3,hashlib,copy
from pathlib import Path

def main(a):
 from sensor_msgs.msg import PointCloud2
 from rclpy.serialization import deserialize_message
 con=sqlite3.connect('file:'+str(a.bag.resolve())+'?mode=ro&immutable=1',uri=True);a.private.mkdir(parents=True,exist_ok=True);jobs=copy.deepcopy(json.loads(a.bindings.read_text()));count=0
 for j in jobs:
  if j['window']!=a.window:continue
  for c in j['clouds']:
   blob=con.execute('select data from messages where id=?',(c['message_id'],)).fetchone()[0];assert hashlib.sha256(blob).hexdigest()==c['cdr_sha'];msg=deserialize_message(blob,PointCloud2);raw=bytes(msg.data);assert hashlib.sha256(raw).hexdigest()==c['raw_sha'];p=a.private/(str(c['message_id'])+'.raw');p.write_bytes(raw);c['raw_path']=str(p);count+=1
 a.output.write_text(json.dumps(jobs));print('verified raw clouds',count)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--window',default='normal');main(p.parse_args())
