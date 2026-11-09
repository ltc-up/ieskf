"""Fair native-time engineering reference comparison of actual causal branches.

Reuses AT's GPS1/CartConvert/full-attitude lever semantics. No fit or estimator
feedback. Historical CSV validity and extended raw-reference masks are separate.
"""
import argparse,csv,json,subprocess,collections
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
from ndt_global.track_evaluation import interpolate_reference,paired_errors
from ndt_global.validation import static_transform
from ndt_global.footprint_audit import antenna_to_base,project_footprint,polygon_audit
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.state import State

def stats(v):
 v=np.asarray(v,float);v=v[np.isfinite(v)]
 return dict(n=len(v),mean=float(v.mean()),rmse=float(np.sqrt(np.mean(v*v))),p50=float(np.median(v)),p95=float(np.percentile(v,95)),p99=float(np.percentile(v,99)),max=float(v.max()),min=float(v.min())) if len(v) else dict(n=0)
def poses(tx,lever):
 pre=[];post=[];Rs=[]
 for r in tx:
  x=np.asarray(r['prior']['p']);R=np.asarray(r['prior']['R']);d=np.asarray(r['update'].get('delta',np.zeros(15))) if r['update']['accepted'] else np.zeros(15);Q=R@Rotation.from_rotvec(d[6:9]).as_matrix();pre.append(x-R@lever);post.append(x+d[:3]-Q@lever);Rs.append(Q)
 return np.asarray(pre),np.asarray(post),np.asarray(Rs)
def reference(binding,ns):
 raw=json.loads(Path(binding['private'],'reference_bound.json').read_text());config=yaml.safe_load(Path(binding['historical_gnss_config']).read_text());config=config.get('/**',{}).get('ros__parameters',config);origin=int(ns[0]);qtime=(ns-origin)*1e-9;atrows=raw['/vectornav/raw/attitude'];at=np.array([v['header']-origin*1e-9 for v in atrows]);q=np.array([v['q'] for v in atrows]);keep=np.r_[True,np.diff(at)>0];at=at[keep];q=q[keep];T=static_transform(raw['/tf_static'],'base_link','vectornav_ant1');R=Rotation.from_euler('xyz',config['pose_orientation_bias'])*Rotation.from_quat([2**-.5,2**-.5,0,0])*Rotation.from_quat(q)*Rotation.from_quat([1,0,0,0])*Rotation.from_matrix(T[:3,:3]).inv();spl=Slerp(at,R);Rm=spl(np.clip(qtime,at[0],at[-1])).as_matrix();gps=[v for v in raw['/vectornav/raw/gps'] if v['fix']==8];gt=np.array([v['header']-origin*1e-9 for v in gps]);payload=''.join(' '.join(map(str,v['lla']))+'\n' for v in gps);result=subprocess.run(['CartConvert','-l',*map(str,config['reference_point']),'-p','10'],input=payload,text=True,capture_output=True,check=True);enu=np.loadtxt(result.stdout.splitlines());hist=list(csv.DictReader(Path(binding['gnss']).open()));ht=np.array([float(v['timestamp'])-origin*1e-9 for v in hist]);hv=np.array([int(v['truth_valid']) and int(v['attitude_bracket_valid']) for v in hist],bool);out={};j=np.searchsorted(at,qtime);jj=np.clip(j,1,len(at)-1);attok=(j>0)&(j<len(at))&(at[jj]-at[jj-1]<=.05)
 for delay in [0.,.075,.150]:
  antenna,ok=interpolate_reference(gt-delay,enu,np.ones(len(gt),bool),qtime,.600001);ok&=attok;G=antenna_to_base(antenna,Rm,T[:3,3]);_,hmask=interpolate_reference(ht,np.zeros(len(ht)),hv,qtime+delay-.075,.100001);out[str(delay)]=dict(G=G,raw_valid=ok,historical_valid=ok&hmask,R=Rm)
 return out

def evaluate(a):
 a.private.mkdir(parents=True,exist_ok=True);motion=json.loads(a.motion.read_text());binding=json.loads(a.at_bindings.read_text());tx=json.loads((a.run/'transactions.json').read_text());base_tx=json.loads((a.baseline/'transactions.json').read_text())[:len(tx)];ns=np.array([r['ref'] for r in tx],np.int64);assert [r['ref'] for r in base_tx]==ns.tolist();n=len(tx);frozen=np.load(a.as_evaluation);assert len(frozen['corner'])>=n;corner=frozen['corner'][:n];tan=frozen['tangent'][:n];lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3];pre,A,RA=poses(tx,lever);_,F,RF=poses(base_tx,lever);pose_ev=[e for e in motion['events'] if e['kind']=='pose'];origin=int(ns[0]);et=(np.array([e['header_ns'] for e in pose_ev],np.int64)-origin)*1e-9;t=(ns-origin)*1e-9;B,bok=interpolate_reference(et,np.array([e['position'] for e in pose_ev]),np.ones(len(et),bool),t,.1);BRot=Slerp(et,Rotation.from_quat([e['quaternion_xyzw'] for e in pose_ev]))(np.clip(t,et[0],et[-1])).as_matrix();refs=reference(binding,ns);summaries={};private={};yaws={'candidate':Rotation.from_matrix(RA).as_euler('xyz')[:,2],'baseline':Rotation.from_matrix(RF).as_euler('xyz')[:,2],'old_EKF':Rotation.from_matrix(BRot).as_euler('xyz')[:,2]}
 def metric(P,G,ok,Rname,RR):
  es,ed=paired_errors(P,G,tan);dy=np.angle(np.exp(1j*(yaws[Rname]-Rotation.from_matrix(RR).as_euler('xyz')[:,2])));return dict(XY=stats(np.linalg.norm((P-G)[ok,:2],axis=1)),abs_longitudinal=stats(abs(es[ok])),signed_lateral=stats(ed[ok]),abs_lateral=stats(abs(ed[ok])),yaw_reference_abs_deg=stats(abs(np.rad2deg(dy[ok]))),Z_difference=stats((P-G)[ok,2]))
 for delay,ref in refs.items():
  G=ref['G'];raw=ref['raw_valid']&bok;historical=ref['historical_valid']&bok;result={}
  for convention,mask in [('historical_common_mask',historical),('raw_extended_reference',raw)]:
   row=dict(valid=int(mask.sum()),coverage=float(mask.mean()),**{name:metric(P,G,mask,name,ref['R']) for name,P in [('candidate',A),('baseline',F),('old_EKF',B)]});corners=[]
   for k in sorted(set(corner)):
    m=mask&(corner==k)
    if not m.any():continue
    corners.append(dict(id=int(k),samples=int(m.sum()),**{name:metric(P,G,m,name,ref['R']) for name,P in [('candidate',A),('baseline',F),('old_EKF',B)]}))
   row['corners']=corners;result[convention]=row
  summaries[delay]=result;private['G_'+delay]=G;private['raw_valid_'+delay]=raw;private['historical_valid_'+delay]=historical
 accepted=np.array([r['update']['accepted'] for r in tx]);unit_gain=[];local_sensitivity=[];precision=[];latent_trace=[];innovation=[];delta=[];last=None;gaps=[];noage=[]
 for r in tx:
  if r['update']['accepted']:
   if last is not None:gaps.append((r['ref']-last)*1e-9)
   last=r['ref'];x=State(np.asarray(r['prior']['p']),np.zeros(3),np.asarray(r['prior']['R']),np.zeros(3),np.zeros(3));raw_residual,H=ExternalPose(lever,('x','y','z','rx','ry','rz')).residual(x,np.asarray(r['ndt']['final']));lat=x.R[:,1];e=np.r_[lat,np.zeros(3)];U=np.asarray(r['ndt'].get('av_projection',{}).get('matrix',np.eye(6)));unit_gain.append(float(e@H@np.asarray(r['update']['K'])@U@e));innovation.append(float(lat@raw_residual[:3]));
   if 'local_residual_jacobian' in r['update']:
    local_sensitivity.append(float(e@H@np.asarray(r['update']['local_residual_jacobian'])@U@e));precision.append(r['update']['expected_precision']);latent_trace.append(r['update']['latent_between_mean_covariance_trace'])
   i=r['index'];delta.append(float(lat@(A[i]-pre[i])))
  noage.append((r['ref']-(last if last is not None else ns[0]))*1e-9)
 states=json.loads((a.run/'states.json').read_text());Ps=np.asarray([r['P'] for r in states]);mineig=np.linalg.eigvalsh(Ps)[:,0];own_seed=max(float(np.max(abs(np.asarray(r['seed'])[:3,3]-(np.asarray(r['prior']['p'])-np.asarray(r['prior']['R'])@lever)))) for r in tx);N=np.array([r['ndt']['final'] for r in tx]);Nbase=np.array([r['ndt']['final'] for r in base_tx]);probe_counts=collections.Counter(r['ndt'].get('av_integrity',{}).get('reason','NOT_EXERCISED') for r in tx);h=json.loads((a.run/'health.json').read_text());runmeta=json.loads(a.run_summary.read_text());main=summaries['0.075']['historical_common_mask'];criteria=dict(complete=n==a.expected,all_numerical_P_positive=bool(mineig.min()>0),both_difficult_corners_improve=all(next((c['candidate']['abs_lateral']['p95']<c['baseline']['abs_lateral']['p95'] for c in main['corners'] if c['id']==k),False) for k in [3,9]),no_other_corner_P95_degradation=all(c['candidate']['abs_lateral']['p95']<=c['baseline']['abs_lateral']['p95'] for c in main['corners'] if c['id'] not in [3,9]),global_XY_not_worse=main['candidate']['XY']['rmse']<=main['baseline']['XY']['rmse'],no_LOST=not any(r['state']=='LOST' for r in h))
 out=dict(label=a.label,processed_frames=n,expected_frames=a.expected,input_coverage=n/a.expected,duration_s=float(t[-1]),actual_CUDA=True,actual_Deskew=True,fixed_NDT_replay=False,own_seed_max_error=own_seed,reference_qualification='GNSS engineering only; raw extension beyond historical CSV uses same frozen AT physical conversion, not certified truth',sensitivity=summaries,accepted=int(accepted.sum()),rejected=int((~accepted).sum()),acceptance_ratio=float(accepted.mean()),decisions=dict(collections.Counter(r['update']['reason'] for r in tx)),health=dict(collections.Counter(r['state'] for r in h)),max_gap_s=max(gaps,default=0),max_recorded_no_observation_age_s=max(noage,default=0),unit_lateral_gain=stats(unit_gain),lateral_innovation=stats(innovation),actual_body_lateral_update=stats(delta),P_min_eigenvalue=float(mineig.min()),P_symmetry_max=float(np.max(abs(Ps-Ps.transpose(0,2,1)))),Z_span_m=float(np.ptp([r['p'][2] for r in states])),vz_range=stats([r['v'][2] for r in states]),bg_norm=stats([np.linalg.norm(r['bg']) for r in states]),ba_norm=stats([np.linalg.norm(r['ba']) for r in states]),NDT_converged=sum(r['ndt']['converged'] for r in tx),local_integrity=dict(probe_counts),first_source_same=tx[0]['source_sha256']==base_tx[0]['source_sha256'],changed_source_count=sum(r['source_sha256']!=b['source_sha256'] for r,b in zip(tx,base_tx)),NDT_vs_baseline_translation=stats(np.linalg.norm((N-Nbase)[:,:3,3],axis=1)),NDT_vs_baseline_fitness=stats([r['ndt']['fitness']-b['ndt']['fitness'] for r,b in zip(tx,base_tx)]),performance=dict(startup_ms=runmeta['startup_ms'],processing_wall_ms=runmeta['wall_ms'],center_align_ms=stats([r['ndt']['align_wall_ms'] for r in tx]),model_Header_to_complete_ms=stats([(r['complete']-r['ref'])*1e-6 for r in tx]),ROS_realtime_qualified=False),strict_repair_screen=criteria,screen_pass=all(criteria.values()),independent_statistical_accuracy=False)
 if a.geometry:
  geom=json.loads(a.geometry.read_text());body=np.asarray(next(iter(geom['polygons'].values()))[0]);load=lambda path:np.genfromtxt(path,delimiter=',',names=True);L=load(binding['left']);R=load(binding['right']);L=np.c_[L['X'],L['Y']];R=np.c_[R['X'],R['Y']];area=lambda q:abs(np.cross(q,np.roll(q,-1,axis=0)).sum());outer,inner=(L,R) if area(L)>area(R) else (R,L);boundary={}
  for name,P,rot in [('candidate',A,RA),('baseline',F,RF),('old_EKF',B,BRot)]:
   checks=[polygon_audit(project_footprint(body,rot[i],P[i]),outer,inner) for i in range(n)];boundary[name]=dict(tested=n,not_contained=sum(not v['contained'] for v in checks),minimum_visual_box_clearance_m=min(v['boundary_separation'] for v in checks),physical_tyre_safety='NOT_VALIDATED')
  out['conditional_visual_box']=boundary
 out['lateral_gain_semantics']='conditional weighted K; not nonlinear residual derivative' if precision else 'linear Gaussian K';out['local_lateral_residual_derivative']=stats(local_sensitivity);out['posterior_precision']=stats(precision);out['latent_between_mean_covariance_trace']=stats(latent_trace)
 out['per_preselected_lap_75ms_raw_reference']=[]
 for lap_id,(lo,hi) in enumerate([(0,1468),(1468,2820),(2820,4134)],1):
  m=refs['0.075']['raw_valid']&bok&(np.arange(n)>=lo)&(np.arange(n)<hi)
  if m.any():out['per_preselected_lap_75ms_raw_reference'].append(dict(lap=lap_id,valid=int(m.sum()),**{name:metric(P,refs['0.075']['G'],m,name,refs['0.075']['R']) for name,P in [('candidate',A),('baseline',F),('old_EKF',B)]}))
 np.savez_compressed(a.private/'evaluation.npz',header=ns,t=t,corner=corner,tangent=tan,candidate=A,baseline=F,old_EKF=B,prior=pre,accepted=accepted,**private);a.output.write_text(json.dumps(out,indent=2)+'\n')
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 ref=refs['0.075'];mask=ref['historical_valid']&bok;fig,ax=plt.subplots(figsize=(8,10));ax.plot(F[:,0],F[:,1],label='legacy15D');ax.plot(A[:,0],A[:,1],label=a.label);ax.plot(B[:,0],B[:,1],label='oldEKF');ax.plot(ref['G'][mask,0],ref['G'][mask,1],'.',ms=1,label='75ms engineering reference');ax.axis('equal');ax.legend();fig.tight_layout();fig.savefig(a.private/'native_overlay.png');plt.close(fig)
 for k in [1,3,9]:
  m=(corner==k)&mask;fig,ax=plt.subplots(figsize=(10,4))
  for name,P in [('candidate',A),('legacy',F),('old_EKF',B)]:ax.plot(t[m],paired_errors(P,ref['G'],tan)[1][m],label=name)
  ax.set_ylabel('GNSS engineering signed lateral difference m');ax.set_xlabel('native time s');ax.legend();fig.tight_layout();fig.savefig(a.private/f'corner{k}.png');plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['run','baseline','motion','as-evaluation','at-bindings','run-summary','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--geometry',type=Path);p.add_argument('--label',required=True);p.add_argument('--expected',type=int,required=True);evaluate(p.parse_args())
