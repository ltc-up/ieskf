"""Offline reference/coverage analysis; never imported by factor comparison."""
import sys,json,subprocess,hashlib
from pathlib import Path
import numpy as np,yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
from compare import load
from factor_diagnostics import motion,transform
REPO=Path(__file__).resolve().parents[2];sys.path.insert(0,str(REPO/'src'))
from ndt_global.validation import static_transform
from ndt_global.track_evaluation import interpolate_reference
from ndt_global.footprint_audit import antenna_to_base

def stat(a):
 a=np.asarray(a,float);return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()),min=float(a.min())) if len(a) else dict(n=0)
def main(root,existing):
 root=Path(root);old=Path(existing);dest=root/'analysis_complete';dest.mkdir(exist_ok=False);plans=[json.load(open(root/'plan.json')),json.load(open(root/'be_holdout_qualified/plan.json'))];P={x['id']:x for p in plans for x in p['cases']};R={}
 for f in [root/'results.jsonl',root/'be_holdout_qualified/results.jsonl']:
  for r in map(json.loads,open(f)):
   if not r['failures']:R[r['id']]=r
 # Strictly reuse the existing offline engineering GNSS/antenna conversion contract.
 raw=json.load(open(old/'phase_bd_ndt_health_bridge/inputs/research_data/phase_at_gnss_footprint_corners/reference_bound.json'));cfg=yaml.safe_load((old/'phase_be_e0_e4/runtime/offline_reference_config.yaml').read_text());cfg=cfg.get('/**',{}).get('ros__parameters',cfg);origin=min(c['t'] for c in P.values());ar=raw['/vectornav/raw/attitude'];at=np.array([r['header']-origin*1e-9 for r in ar]);quat=np.array([r['q'] for r in ar]);keep=np.r_[True,np.diff(at)>0];at=at[keep];quat=quat[keep];T=static_transform(raw['/tf_static'],'base_link','vectornav_ant1');rot=Rotation.from_euler('xyz',cfg['pose_orientation_bias'])*Rotation.from_quat([2**-.5,2**-.5,0,0])*Rotation.from_quat(quat)*Rotation.from_quat([1,0,0,0])*Rotation.from_matrix(T[:3,:3]).inv();slerp=Slerp(at,rot);gps=[r for r in raw['/vectornav/raw/gps'] if r['fix']==8];gt=np.array([r['header']-origin*1e-9 for r in gps]);payload=''.join(' '.join(map(str,r['lla']))+'\n' for r in gps);z=subprocess.run(['CartConvert','-l',*map(str,cfg['reference_point']),'-p','10'],input=payload,text=True,capture_output=True,check=True);enu=np.loadtxt(z.stdout.splitlines())
 def ref(t,lag):
  qt=np.array([(t-origin)*1e-9]);assert at[0]<=qt[0]<=at[-1];rot=slerp(qt).as_matrix();ant,valid=interpolate_reference(gt-lag,enu,np.ones(len(gt),bool),qt,.600001)
  if not valid[0]:return None
  ans=np.eye(4);ans[:3,:3]=rot[0];ans[:3,3]=antenna_to_base(ant,rot,T[:3,3])[0];return ans
 def relative_ref(a,b,lag):
  aa=ref(a,lag);bb=ref(b,lag);return None if aa is None or bb is None else np.linalg.inv(aa)@bb
 def error(A,B):
  if B is None:return dict(valid=False,reason='ORIGINAL_REFERENCE_VALIDITY_GAP')
  D=np.linalg.inv(B)@A;return dict(translation_local=D[:3,3].tolist(),translation_norm=float(np.linalg.norm(D[:3,3])),xy=float(np.linalg.norm(D[:2,3])),lateral=float(D[1,3]),longitudinal=float(D[0,3]),yaw_deg=float(np.rad2deg(np.arctan2(D[1,0],D[0,0]))),rotation_deg=float(np.rad2deg(Rotation.from_matrix(D[:3,:3]).magnitude())))
 detail=[];factors=[]
 for id,r in R.items():
  c=P[id];p=load(c);hmap={plans[0]['identities'][h['source']]['sha256']:h for h in c['history']};n=dict(id=id,group=c['group'],A_correction_xy=float(np.linalg.norm(np.array(c['map_result']['final'])[:2,3]-np.array(c['seed'])[:2,3])),A={},B=[],C=[],D={},comparison_wall_ms=r['comparison_wall_ms'],source_n=len(p))
  for lag in [0.,.075,.15]:n['A'][str(lag)]=error(np.array(c['map_result']['final']),ref(c['t'],lag))
  for method in ['B','C']:
   for b in r[method]:
    if 'coverage' not in b:continue
    h=hmap[b['target_identity']];hp=load(h);E=None
    # Reuse E already saved for B; C's E is reconstructed from the same preregistered causal inputs.
    if method=='B':E=np.array(b['E_relative'])
    else:
     if 'motion_data' not in locals():motion_data=json.load(open(plans[0]['IMU_Wheel']));I=sorted([e for e in motion_data['events'] if e['kind']=='imu' and e.get('valid')],key=lambda e:e['header_ns']);W=sorted([e for e in motion_data['events'] if e['kind']=='wheel' and e.get('valid')],key=lambda e:e['header_ns'])
     E=motion(h['t'],c['t'],I,W,np.array(plans[0]['R_base_imu']))
    distance,_=cKDTree(hp).query(transform(E,p),workers=1);b0=dict(method=method,dt_ms=b['dt_ms'],voxel_coverage=b['coverage'],nearest_neighbor_overlap={str(rad):float((distance<rad).mean()) for rad in [.5,1,2]},factor_ms=b['factor_ms'],weak_direction=b['weak_direction_XY_Lyaw'],schur_eigenvalues=b['source_nuisance_schur_XY_Lyaw_eigenvalues'],GN_candidate=b['source_local_GN_candidate'],mean_cost=b['mean_cost'],E={})
    for lag in [0.,.075,.15]:b0['E'][str(lag)]=error(E,relative_ref(h['t'],c['t'],lag))
    if 'map_feedback_branch_control' in b:b0['map_control']=dict(coverage=b['map_feedback_branch_control']['coverage']['fraction'],cost=b['map_feedback_branch_control']['mean_cost'])
    n[method].append(b0);factors.append(b0)
  if r['D']:
   d=r['D'];anchor=c['t']-int(round(max(d['history_age_ms'])*1e6));n['D']=dict(converged=d['result']['converged'],valid=d['result']['candidate_valid'],exit=d['result']['exit'],wall_ms=d['wall_ms'],history_age_ms=d['history_age_ms'],E={},candidate={})
   for lag in [0.,.075,.15]:
    refrel=relative_ref(anchor,c['t'],lag);n['D']['E'][str(lag)]=error(np.array(d['E_relative']),refrel);n['D']['candidate'][str(lag)]=error(np.array(d['result']['final']),refrel)
  detail.append(n)
 (dest/'cases.json').write_text(json.dumps(detail,indent=2));summary=dict(cases=len(detail),B_evaluated=sum(len(n['B'])for n in detail),C_evaluated=sum(len(n['C'])for n in detail),D_alignments=sum(bool(n['D'])for n in detail),missing_nominal_B=sum('coverage' not in b for r in R.values()for b in r['B']),by_interval={},D_performance=stat([n['D']['wall_ms']for n in detail if n['D']]),factor_ms=stat([n['factor_ms']for n in factors]),comparison_ms=stat([n['comparison_wall_ms']for n in detail]),RSS_peak_KiB=max(r['RSS_peak_KiB']for r in R.values()),GPU_memory='UNKNOWN: no per-call peak measurement; B/C CPU only',D_over50=sum(n['D']['wall_ms']>50 for n in detail if n['D']),D_75ms_relative_xy_better_than_E=sum(n['D']['candidate']['0.075']['xy']<n['D']['E']['0.075']['xy'] for n in detail if n['D'] and 'xy' in n['D']['candidate']['0.075'] and 'xy' in n['D']['E']['0.075']),C_native_GPU_or_joint_optimization_run=False,reference='existing offline engineering GNSS/INS+antenna-to-base, 0/75/150ms; NOT independent ground truth; no reference enters compare.py')
 for lag in [50,100,150]:
  b=[x for n in detail for x in n['B'] if abs(x['dt_ms']-lag)<=5];summary['by_interval'][str(lag)]=dict(voxel_coverage=stat([x['voxel_coverage']['fraction']for x in b]),NN_1m_coverage=stat([x['nearest_neighbor_overlap']['1']for x in b]),E_75ms_relative_xy=stat([x['E']['0.075']['xy']for x in b if 'xy' in x['E']['0.075']]))
 (dest/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main(*sys.argv[1:])
