"""Offline diagnostic replay. Private inputs are supplied explicitly, never distributed."""
import argparse,json,hashlib,time,os
from pathlib import Path
import numpy as np
from geometry_ieskf.replay import EventReplay
from geometry_ieskf.instrumented import InstrumentedUpdate
from geometry_ieskf.geometry import GeometryCorrespondence
from geometry_ieskf.common import dump,sha

def run():
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--contract',required=True);p.add_argument('--source-root',required=True);p.add_argument('--output',required=True);p.add_argument('--baseline');p.add_argument('--target');p.add_argument('--legacy-index');p.add_argument('--legacy-root');p.add_argument('--legacy-binding');p.add_argument('--frozen-reference');a=p.parse_args()
 data=json.load(open(a.input));contract=json.load(open(a.contract));root=Path(a.source_root);out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
 target_path=a.target or contract['target_full'];assert sha(target_path)==contract['target_sha256']
 legacy=None;binding=None
 if a.legacy_index:
  legacy={int(row['stamp_ns']):row for row in map(json.loads,Path(a.legacy_index).read_text().splitlines())};ts=np.array(sorted(legacy))
 if a.legacy_binding:binding=json.load(open(a.legacy_binding))['source_sha256']
 mappings=[]
 def load(e):
  if legacy is None:
   path=root/e['source_file'];assert sha(path)==e['source_sha256'];return np.load(path)
  stamp=int(ts[np.argmin(abs(ts-e['common_header_ns']))]);assert abs(stamp-e['common_header_ns'])<=1000
  row=legacy[stamp];path=Path(a.legacy_root)/(str(stamp)+'.bin')
  assert row['input_frame']=='base_link'
  assert binding is not None and sha(path)==binding[str(stamp)]
  xyz=np.fromfile(path,dtype='<f4',offset=8).reshape(-1,3).astype(float);T=np.asarray(e['T_native_from_common'])
  mappings.append({'native_ns':e['header_ns'],'legacy_common_ns':stamp,'new_common_ns':e['common_header_ns'],'legacy_sha256':sha(path),'rebase':e['T_native_from_common']})
  return xyz@T[:3,:3].T+T[:3,3]
 target=np.fromfile(target_path,dtype='<f4',offset=8).reshape(-1,3).astype(float);query=GeometryCorrespondence(target)
 replay=EventReplay(contract,source_loader=load);replay.solver=InstrumentedUpdate(replay.factor)
 # Attach observations after each complete call without changing any decision.
 original=replay.solver.correct
 def diagnostic(*args):
  x,P,info=original(*args);info['solver_trace']=replay.solver.trace;return x,P,info
 replay.solver.correct=diagnostic
 if a.frozen_reference:
  from geometry_ieskf.state import State
  from scipy.spatial.transform import Rotation
  reference=json.load(open(a.frozen_reference));frames={e['header_ns']:e for e in data['frames']};result=[]
  for row in reference['lidar_updates']:
   st=row['predicted'];x=State(np.array(st['p_imu_map']),np.array(st['v_imu_map']),Rotation.from_quat(st['q_map_body_xyzw']).as_matrix(),np.array(st['bg_body_rad_s']),np.array(st['ba_body_m_s2']),st['state_time_ns']);P=np.array(st['P']);e=frames[row['header_ns']]
   y,post,info=diagnostic(x,P,load(e),query);result.append({'header_ns':row['header_ns'],'corrected':replay.diagnostics.snapshot(y,post),**info})
  dump(out,result)
  print('Frozen common-prior evaluations',len(result),flush=True)
  return
 result=replay.run(data,query);dump(out,result)
 if legacy is not None:dump(out.with_suffix('.sources.json'),mappings)
 if a.baseline:
  old=json.load(open(a.baseline));assert result['counts']==old['counts'];assert len(result['outputs'])==len(old['outputs'])
  errors={k:max(float(np.max(abs(np.asarray(x[k])-np.asarray(y[k])))) for x,y in zip(result['outputs'],old['outputs'])) for k in ['p_imu_map','v_imu_map','P','bg_body_rad_s','ba_body_m_s2']}
  reasons=all(x['reason']==y['reason'] and x['accepted']==y['accepted'] for x,y in zip(result['lidar_updates'],old['lidar_updates']))
  dump(out.with_suffix('.parity.json'),{'maximum_absolute_errors':errors,'same_all_lidar_decisions':reasons,'pass':reasons and max(errors.values())<1e-9});assert reasons and max(errors.values())<1e-9
 print(result['counts'],result['cpu_replay_wall_s'],flush=True)
if __name__=='__main__':run()
