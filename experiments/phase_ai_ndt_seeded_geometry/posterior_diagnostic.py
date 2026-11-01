import sys,json
from pathlib import Path
import numpy as np
from geometry_ieskf.common import dump
from geometry_ieskf.objective import state
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.qualification import TrackedCorrespondence
from geometry_ieskf.state import inject,chart_map
from geometry_ieskf.posterior import block_posterior
from geometry_ieskf.seeded_geometry import SeededUpdate
import argparse,os
a=argparse.ArgumentParser();a.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));args=a.parse_args();assert args.data_root,'Set IESKF_DATA_ROOT';R=Path(args.data_root);D=R/'phase_ai_ndt_seeded_geometry';x=json.load(open(D/'private_results_16.json'));lever=np.array(json.load(open(R/'history/phase_v/results/input_contract.json'))['extrinsic']['lever_body_imu_m']);factor=GeometryFactor(lever);target=np.fromfile(R/'inputs/map_target.bin',np.float32,offset=8).reshape(-1,3).astype(float);corr=TrackedCorrespondence(target);selected=json.load(open(R/'phase_ah_deskew_z_ablation/selection_private.json'))['selected'];out=[]
for e in x:
 for label,v in e['runs'].items():
  if not v['end']['certificate']['accepted']:continue
  p=state(e['prior']);P=np.array(e['prior']['P']);z=np.array(v['end']['delta']);sel=next(a for a in selected if a['alias']==e['alias']);source=np.load(R/'history/phase_v'/sel['frame']['source_file']);c=corr.get(inject(p,z),source,factor);r,H,_=factor.linearize(inject(p,z),c['p'],c['q'],c['n']);J=H@chart_map(z);w=c['w']*SeededUpdate(factor).robust(r)[1];pose=[0,1,2,6,7,8];q=block_posterior(P,J[:,pose],w);B=chart_map(z);post=B@q['P_chart']@B.T;post=(post+post.T)/2
  def stats(P):return dict(P_ZZ=float(P[2,2]),P_vzvz=float(P[5,5]),P_vzZ=float(P[5,2]),corr_Z_vz=float(P[5,2]/np.sqrt(P[2,2]*P[5,5])),sigma_Z_m=float(np.sqrt(P[2,2])),sigma_vz_m_s=float(np.sqrt(P[5,5])))
  out.append(dict(alias=e['alias'],start=label,prior_summary=stats(P),conditional_posterior_summary=stats(post),numeric_P_positive=bool(np.linalg.eigvalsh(post).min()>0),complete_state_delta=z.tolist(),P_candidate=post.tolist(),state_P_commits=0,STATISTICAL_POSTERIOR_CALIBRATED=False))
dump(D/'posterior_private.json',out);dump(D/'posterior_summary.json',[{k:v for k,v in a.items() if k not in ['complete_state_delta','P_candidate']} for a in out]);print(json.dumps(json.load(open(D/'posterior_summary.json')),indent=2))
