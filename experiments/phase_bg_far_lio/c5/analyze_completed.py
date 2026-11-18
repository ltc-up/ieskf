"""Offline only: no matcher, sensor ingestion, or parameter changes."""
import json,sys,pickle,hashlib,collections,time,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];sys.path[:0]=[str(ROOT/'src'),str(Path(__file__).parent),str(ROOT/'experiments/phase_bg_far_lio')]
from geometry_ieskf.external_pose import ExternalPose
from analyze_windows import stats,interpolate,evaluate

def rows(p):
 for l in Path(p).open():yield json.loads(l)
def main(root):
 root=Path(root);run=root/'eskf15_1013';out=root/'analysis';out.mkdir(exist_ok=False);cfg=json.load(open(root/'config.json'));B=cfg['start_ns']-130_000_000_000;paths=json.load(open(root/'paths.json'));lever=np.array(json.load(open(cfg['extrinsics']))['T_base_imu'])[:3,3]
 # Trusted own checkpoint, reconstruct every event without publisher or passive observer.
 sys.path.insert(0,str(ROOT/'experiments/phase_be_e0_e4'))
 replay=pickle.load(open(run/'deterministic_checkpoint.pkl','rb'));mismatch=[];n=0;t=time.monotonic();reasons=collections.Counter()
 for row in rows(run/'kernel_events.jsonl'):
  e=row['event'];
  if e['kind']=='ndt':e['observer']=ExternalPose(lever,('x','y','z','rx','ry','rz'));e['pose']=np.array(e['pose']);e['R']=np.array(e['R'])
  result=replay.ingest(e);x,P,_=replay.current();h=hashlib.sha256(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba,P.ravel()].astype('<f8').tobytes()).hexdigest();n+=1;reasons[result]+=1
  if h!=row['state_hash'] or result!=row['result']:mismatch.append(n)
  replay.validate()
 parity=dict(events=n,mismatches=mismatch,wall_seconds=time.monotonic()-t,reasons=dict(reasons),scope='saved retained-history checkpoint through end only; no second frontend run')
 (out/'parity.json').write_text(json.dumps(parity,indent=2))
 M=[r for r in rows(run/'matches.jsonl') if 'ndt' in r];fault=[r for r in M if 515<=(r['stamp_ns']-B)/1e9<=525];s={};s['parity']=parity;s['matches']=len(M);s['reasons']=dict(collections.Counter(r['update']['reason'] for r in M));s['fault_reasons']=dict(collections.Counter(r['update']['reason'] for r in fault));s['fault_count']=len(fault)
 s['timing']={k:stats([r[k] for r in M]) for k in ['after_available_ms','ndt_wall_ms','deskew_wall_ms','delayed_update_ms']};s['over50']=sum(r['after_available_ms']>50 for r in M);s['over50_ndt']=sum(r['ndt_wall_ms']>50 for r in M)
 s['fitness']=stats([r['ndt']['fitness'] for r in M]);s['high_fitness_over8']=dict(n=sum(r['ndt']['fitness']>8 for r in M),accepted=sum(r['ndt']['fitness']>8 and r['update'].get('accepted',False) for r in M));s['fault_fitness']=stats([r['ndt']['fitness'] for r in fault]);s['replayed_events']=stats([r['replayed_events'] for r in M]);s['fault_replayed_events']=stats([r['replayed_events'] for r in fault])
 # Existing independent-map engineering comparison, no extrapolation or map fitting.
 old=Path(json.load(open(root/'analysis_inputs.json'))['historical_0105']);rr=list(csv.DictReader(old.open()));tt=np.array([int(r['sensor_stamp_ns']) for r in rr],dtype=np.int64);pp=np.array([[float(r['ndt_'+k]) for k in ['x','y','z']] for r in rr]);yy=np.unwrap([float(r['ndt_yaw']) for r in rr]);records=[]
 for i,r in enumerate(fault):
  t=r['stamp_ns'];N=np.array(r['ndt']['final']);seed=np.array(r['seed']);cur=r['current_after'];pcur=np.array(cur['p'])-np.array(cur['R'])@lever
  z=dict(bag_s=(t-B)/1e9,source_sha=r['source_sha'],accepted=r['update'].get('accepted',False),reason=r['update']['reason'],fitness=r['ndt']['fitness'],converged=r['ndt']['converged'],iterations=r['ndt'].get('iterations'),correction_xy=float(np.linalg.norm(N[:2,3]-seed[:2,3])),delta=r['update'].get('delta'),comparisons={})
  for shift in [0,.075,.15]:
   ref_t=t+int(shift*1e9)
   if not tt[0]<=ref_t<=tt[-1]:continue
   refp=np.array([np.interp(ref_t-B,tt-B,pp[:,j]) for j in range(3)]);yaw=np.interp(ref_t-B,tt-B,yy);u=np.array([np.cos(yaw),np.sin(yaw)]);normal=np.array([-u[1],u[0]])
   def err(p):
    e=p[:2]-refp[:2];return dict(xy=float(np.linalg.norm(e)),longitudinal=float(u@e),lateral=float(normal@e))
   # Current state is evaluated at its own timestamp, separately.
   tc=cur['t']+int(shift*1e9);refc=np.array([np.interp(tc-B,tt-B,pp[:,j]) for j in range(3)]) if tt[0]<=tc<=tt[-1] else None
   z['comparisons'][str(shift)]=dict(seed=err(seed[:3,3]),ndt=err(N[:3,3]),current_xy=None if refc is None else float(np.linalg.norm(pcur[:2]-refc[:2])))
  records.append(z)
 (out/'fault_cases.json').write_text(json.dumps(records,indent=2))
 s['fault_engineering']={}
 for shift in ['0','0.075','0.15']:
  q=[r['comparisons'][shift] for r in records if shift in r['comparisons']];s['fault_engineering'][shift]={kind:{axis:stats([r[kind][axis] for r in q]) for axis in ['xy','longitudinal','lateral']} for kind in ['seed','ndt']};s['fault_engineering'][shift]['current_xy']=stats([r['current_xy'] for r in q if r['current_xy'] is not None])
 s['fault_over3m_0105']=[r['bag_s'] for r in records if r['comparisons'].get('0',{}).get('ndt',{}).get('xy',0)>3]
 s['fault_first_last']=[records[0],records[-1]];s['fault_max_case']=max(records,key=lambda r:r['comparisons']['0']['ndt']['xy'])
 # Original frozen scoring grid and existing 0/75/150ms GNSS engineering reference.
 be=ROOT.parent;ref=np.load(be/'runtime/reference_lap.npz');ev=np.load(Path(paths['data'])/'phase_as_accuracy_quality/single_lap/evaluation.npz');grid=ref['header'];S=list(rows(run/'states.jsonl'));st=np.array([r['stamp_ns'] for r in S]);pos=np.array([r['position'] for r in S]);rot=Rotation.from_quat([r['quaternion'] for r in S]).as_matrix();T=np.array([r['ndt']['final'] for r in M]);mt=np.array([r['stamp_ns'] for r in M]);s['scoring_windows']={}
 for name,lo,hi in [('normal8',675,803),('right3',1317,1405)]:
  ids=np.arange(lo,hi+1);g=grid[ids];a=interpolate(st,pos,rot,g);b=interpolate(mt,T[:,:3,3],T[:,:3,:3],g);s['scoring_windows'][name]={k:evaluate(v[0],v[1],ids,ref,ev['tangent'],v[2]&(v[3]<100_000_000)) for k,v in [('state',a),('ndt_interpolated',b)]}
 # Actual output consistency, not an average-frequency proxy.
 amin=1e99;asym=0.;bad=0;seen=0;packet_example=None
 for r in rows(run/'output100.jsonl'):
  p=r['packet']
  if p.get('valid'):
   if packet_example is None:packet_example=list(p)
   Q=np.array(p['P']);amin=min(amin,float(np.linalg.eigvalsh(Q).min()));asym=max(asym,float(np.abs(Q-Q.T).max()));bad+=int(not np.isfinite(Q).all());seen+=1
 s['output_P']=dict(checked=seen,min_eigenvalue=amin,max_asymmetry=asym,nonfinite=bad,packet_fields=packet_example)
 s['run_status']=json.load(open(run/'run_status.json'));s['output_summary']=json.load(open(run/'output100_summary.json'));s['resources']=json.load(open(run/'resource_summary.json'));s['reference_limit']='fault uses independent-map historical NDT trajectory, not GNSS ground truth; plus-time sensitivity explicitly not GNSS accuracy calibration'
 (out/'summary.json').write_text(json.dumps(s,indent=2));print(json.dumps({k:s[k] for k in ['matches','reasons','fault_count','fault_reasons','timing','over50','high_fitness_over8','fault_engineering','output_P']},indent=2))
if __name__=='__main__':main(sys.argv[1])
