"""Private identity-based preparation; never writes original bag/audit files."""
import argparse,json,sqlite3,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from pathlib import Path
from scipy.spatial.transform import Rotation,Slerp
from geometry_ieskf.common import dump,sha

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--audit-root',required=True);ap.add_argument('--motion-file',required=True);ap.add_argument('--bag',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();D=Path(a.data_root);H=Path(a.audit_root);O=Path(a.output);aa=json.load(open(D/'phase_aa_z_validation/new_replay.json'));events=aa['lidar_updates'];manifest=json.load(open(D/'phase_aa_z_validation/INPUT_MANIFEST.json'));bind={x['header']:x for x in json.load(open(H/'registration_sequence_20261101/stats/input_binding.json'))};cases={x['header']:x for x in json.load(open(H/'registration_sequence_20261101/stats/prepared_cases.json'))};mr=[json.loads(l) for l in open(a.motion_file) if json.loads(l).get('pose_emitted')];ns=np.array([r['stamp_ns'] for r in mr]);frames={f['header_ns']:f for f in manifest['source_frames']};metrics=[]
 for i,e in enumerate(events):
  k=int(np.argmin(abs(ns-e['header_ns'])));row=mr[k];j=bind[e['historical_common_header_ns']];b=np.fromfile(j['pose_path'],'<f8').reshape(-1,8);span=Slerp(b[:,0],Rotation.from_quat(b[:,1:5]))([min(c['scan_start_ns'] for c in j['clouds'])*1e-9-j['native_reference_ns']*1e-9,0]).as_matrix();rp=Rotation.from_matrix(span[0].T@span[1]).as_rotvec();metrics.append(dict(index=i,yaw_rate=row['yaw_rate'],speed=row['speed_mps'],RP_change_rad=float(np.linalg.norm(rp[:2]))))
 roles={'first':0,'max_rejected_raw_vz':max((max((abs(p['absolute_frozen_delta'][5]) for p in e['solver_trace']['proposals']),default=0),i) for i,e in enumerate(events) if not e['accepted'])[1],'max_old_accepted_vz':max((abs(e['delta'][5]),i) for i,e in enumerate(events) if e['accepted'])[1],'least_curvature_converged':min((abs(metrics[i]['yaw_rate']),i) for i,e in enumerate(events) if e['accepted'] and e['reason']=='CONVERGED')[1],'max_speed_yaw':max((m['speed']*abs(m['yaw_rate']),m['index']) for m in metrics)[1],'max_scan_RP':max((m['RP_change_rad'],m['index']) for m in metrics)[1]};con=sqlite3.connect('file:'+str(Path(a.bag).resolve())+'?mode=ro&immutable=1',uri=True);selected=[];jobs=[]
 for num,i in enumerate(sorted(set(roles.values()))):
  e=events[i];f=frames[e['header_ns']];j=bind[e['historical_common_header_ns']];c=cases[e['historical_common_header_ns']];assert j['gates_pass'];assert j['native_reference_ns']==e['header_ns'];assert e['predicted']['state_time_ns']==e['header_ns']
  for original,cloud in zip(f['source_cloud_identities'],j['clouds']):
   row=con.execute('select data from messages where id=?',(original['message_id'],)).fetchone();assert row and __import__('hashlib').sha256(row[0]).hexdigest()==original['cdr_sha']==cloud['cdr_sha'];m=deserialize_message(row[0],PointCloud2);raw=bytes(m.data);assert __import__('hashlib').sha256(raw).hexdigest()==cloud['raw_sha'];path=O/(str(e['header_ns'])+'_'+str(original['message_id'])+'.raw');path.write_bytes(raw);cloud['raw_path']=str(path.resolve())
  assert sha(D/'history/phase_v'/f['source_file'])==e['source_sha256'];j=dict(j);j['scene']='AH_frozen';j['gates_pass']=True;jobs.append(j);selected.append(dict(alias='E'+str(num+1),AA_index=i,roles=[r for r,n in roles.items() if n==i],event=e,frame=f,job=j,case=c,motion=metrics[i]))
 dump(O/'jobs.json',jobs);dump(O/'selection_private.json',dict(selected=selected,roles=roles,motion_cache_sha256=sha(Path(a.motion_file)),raw_bag_path=str(Path(a.bag).resolve()),metadata_sha256=sha(Path(a.bag).parent/'metadata.yaml'),all_CDR_and_saved_raw_verified=True));print('Selected',[(x['alias'],x['AA_index'],x['roles']) for x in selected])
if __name__=='__main__':main()
