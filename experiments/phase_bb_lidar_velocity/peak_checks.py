"""Sensor-disagreement-selected fixed pair diagnostics; not error labels."""
import argparse,json,subprocess,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.lidar_velocity import body_twist
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--matcher',type=Path,required=True);a=p.parse_args();rows=json.loads((a.root/'frozen_velocity.private.json').read_text());rows=sorted(rows,key=lambda z:np.linalg.norm(np.array(z['sensor_prediction'])-z['optical']),reverse=True);chosen=[]
for r in rows:
 if all(abs(r['index']-q['index'])>10 for q in chosen):chosen.append(r)
 if len(chosen)==8:break
meta={r['header']:r for r in map(json.loads,Path('data/phase_ar_loop44_map0105/fullpose100_lap/deskew_meta_stream.jsonl').read_text().splitlines())};tx=json.loads(Path('data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json').read_text());out=[];proc=subprocess.Popen([str(a.matcher)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.root/'peak_checks.stderr').open('w'),text=True)
for row in chosen:
 i,lag=row['index'],row['lag'];prev,cur=tx[i-lag],tx[i];dt=(cur['ref']-prev['ref'])*1e-9;seed=np.linalg.inv(prev['seed'])@np.array(cur['seed']);results=[]
 for name,dx,dy,yaw in [('nominal',0,0,0),('x_plus',1,0,0),('x_minus',-1,0,0),('y_plus',0,.5,0),('y_minus',0,-.5,0),('yaw_plus',0,0,.03),('reverse',0,0,0)]:
  T=seed.copy();T[:3,3]+=[dx,dy,0];T[:3,:3]=T[:3,:3]@Rotation.from_euler('z',yaw).as_matrix();source=meta[cur['ref']]['source_path'];target=meta[prev['ref']]['source_path']
  if name=='reverse':T=np.linalg.inv(T);source,target=target,source
  q=dict(case_id=name,source_path=source,target_path=target,target_sha=hashlib.sha256(target.encode()).hexdigest(),seed_common=T.tolist(),roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0]);proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();r=json.loads(proc.stdout.readline());r['test']=name
  if r.get('candidate_valid'):r['velocity']=body_twist(np.linalg.inv(r['final']) if name=='reverse' else r['final'],dt).tolist()
  results.append(r)
 out.append(dict(index=i,lag=lag,selection='largest optical disagreement, separated10frames, no GNSS label',runs=results))
proc.stdin.close();proc.wait(timeout=30);(a.root/'peak_checks.private.json').write_text(json.dumps(out));summary=[]
for x in out:
 v=np.array([r['velocity'] for r in x['runs'] if r.get('converged') and 'velocity' in r]);summary.append(dict(lag=x['lag'],converged=int(len(v)),tests=len(x['runs']),velocity_range_m_s=np.ptp(v[:,:3],axis=0).tolist() if len(v) else None,known_correct_peak=False))
(a.root/'peak_checks.json').write_text(json.dumps(summary,indent=2)+'\n')
