"""Additional explicitly conditional offline historical branch window.
Uses original causal Seeds and archived sources, never reference trajectories.
No claim to reconstruct historical EKF velocity/Bias/P or LiDAR recorder time.
"""
import json,sys,hashlib
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);old=Path(sys.argv[2]);cfg=json.load(open(root/'inputs/series.json'));motion=json.load(open(cfg['motion']))['events'];request=json.load(open(old/'phase_bg_far_lio/c4/frozen/requests.json'));by={}
for q in request:
 t=q['header'];bag=q['identity']['bag_relative']
 if 517.93<=bag<=518.39 and q['identity']['map_variant']in ['1013','1013_captured_raw']:by.setdefault(t,q)
rows=[]
for t,q in sorted(by.items()):
 rows.append(dict(t=t,available=max([t]+[e['recorder_ns']for e in motion if e['kind']in ['imu','wheel']and e.get('valid')and t-1_000_000_000<=e['header_ns']<=t]),source=q['source_path'],source_sha=hashlib.sha256(Path(q['source_path']).read_bytes()).hexdigest(),source_n=q['identity']['source_n'],seed=q['seed_common'],source_contract='original q11 production Deskew and original causal Seed; offline motion release lower bound, original LiDAR arrival UNKNOWN'))
assert len(rows)==5
T=np.array(cfg['T_base_imu']);Rb=T[:3,:3];lever=T[:3,3];r=rows[0];t=r['t'];seed=np.array(r['seed']);R=seed[:3,:3];wheel=max((e for e in motion if e['kind']=='wheel'and e.get('valid')and e['header_ns']<=t),key=lambda e:e['header_ns']);im=max((e for e in motion if e['kind']=='imu'and e.get('valid')and e['header_ns']<=t),key=lambda e:e['header_ns']);v=R@(np.array([wheel['speed'],0.,0.])+np.cross(Rb@im['omega'],lever));sigma=np.repeat(cfg['noise']['initial_std_p_v_theta_bg_ba'],3);r['prior']=dict(p=(seed[:3,3]+R@lever).tolist(),R=R.tolist(),v=v.tolist(),bg=[0.,0.,0.],ba=[0.,0.,0.],P=np.diag(sigma**2).tolist());cfg['series']=[dict(id='old_branch',map_key='map1013',records=rows,qualification='conditional offline5scan graph, not recovered mature historical state; zeroBias/defaultP/initial lateral velocity assumptions explicit')];(root/'inputs/old_conditional_window.json').write_text(json.dumps(cfg,indent=2));print('frozen original branch window',len(rows),'sources')
