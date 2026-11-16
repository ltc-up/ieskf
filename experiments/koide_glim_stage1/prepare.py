"""Freeze identities/selection before factor evaluation, never reference-outcome selection."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[2]
def main(root,existing):
 root=Path(root);old=Path(existing);c4=old/'phase_bg_far_lio/c4';sel=json.load(open(c4/'frozen/selection_before_matching.json'))['items'];req=json.load(open(c4/'frozen/requests.json'));result={r['request']['case_id']:r['result'] for r in map(json.loads,open(c4/'frozen/results.jsonl'))};cfg=json.load(open(old/'phase_bg_far_lio/c5/config.json'));bi=np.array(json.load(open(cfg['extrinsics']))['T_base_imu']);cases=[]
 for ix in [18,21,22,23,7,9]:
  it=sel[ix];t=it['header'];parent=Path(it['source']).parent.parent;history=[]
  for p in sorted(parent.iterdir()):
   if p.is_dir() and p.name.isdigit() and 0<t-int(p.name)<=1_000_000_000 and (p/'source_after_preprocess.pcd').exists():history.append(dict(t=int(p.name),source=str(p/'source_after_preprocess.pcd'),format='pcd',seed=str(p/'seed_pose.txt'),final=str(p/'final_pose.txt')))
  cases.append(dict(id='C4_'+str(ix),group=it['group'],t=t,source=it['source'],format='pcd',seed=req[ix]['seed_common'],map_result=result['C4_%02d'%ix],history=history,map_variant='1013',source_contract='original historical cropped/deskewed base scan; original map feedback history, NOT independent deskew'))
 folder=old/'phase_be_e0_e4/results/controlled2_matched_lap';frames=list(map(json.loads,open(folder/'frames.jsonl')));lever=bi[:3,3]
 for i,label in [(740,'normal8'),(1360,'right3'),(1120,'historical_large_innovation_proxy'),(130,'historical_large_innovation_proxy'),(200,'ordinary_control'),(400,'ordinary_control')]:
  r=frames[i];b=r['branches'][0];x=b['prior'];seed=np.eye(4);seed[:3,:3]=x['R'];seed[:3,3]=np.array(x['p'])-np.array(x['R'])@lever;t=r['ref'];history=[dict(t=f['ref'],source=str(folder/'deskew_ndt'/f"{f['ref']}_A1.bin"),format='bin',seed=None,final=f['branches'][0]['ndt']['final']) for f in frames[max(0,i-20):i]]
  cases.append(dict(id='BE_'+str(i),group=label,t=t,source=str(folder/'deskew_ndt'/f'{t}_A1.bin'),format='bin',seed=seed.tolist(),map_result=b['ndt'],history=history,map_variant='0105',source_contract='saved own15D CUDA Deskew Source, map-feedback prehistory; no new deskew'))
 identities={}
 for c in cases:
  for p in [c['source']]+[h['source'] for h in c['history']]:
   f=Path(p);identities[p]=dict(exists=f.is_file(),bytes=f.stat().st_size if f.is_file() else None,sha256=hashlib.sha256(f.read_bytes()).hexdigest() if f.is_file() else None)
 plan=dict(baseline='2772172000625397b726efa51939bf561524ff88',cases=cases,identities=identities,IMU_Wheel=str(old/'phase_bd_ndt_health_bridge/inputs/research_data/phase_ao_ap_validation/loop44.json'),R_base_imu=bi[:3,:3].tolist(),matcher=str(old/'phase_bd_ndt_health_bridge/runtime/match_p0'),origins=json.load(open(c4/'input_time_contract.json')),limits=dict(new_D_matches=12,new_joint_optimization=0,increment_MiB=256,RSS_GiB=3),factor=dict(backend='independent_CPU_equation_diagnostic',resolution=.5,neighbors=20,plane_eigenvalues=[.001,1,1],surface_validation=False,L_m=10),B_nominal_ms=[50,100,150],B_tolerance_ms=5,C=dict(max_keyframes=3,max_history_ms=1000,max_overlap=.7,min_overlap=.01),no_online_state_updates=True,no_seed_feedback=True,new_point_cloud_archives=False)
 with (root/'plan.json').open('x') as f:json.dump(plan,f,indent=2)
 print('Frozen',len(cases),'cases;',len(identities),'small source identities;',sum(v['bytes'] or 0 for v in identities.values()),'input bytes reused')
if __name__=='__main__':main(*sys.argv[1:])
