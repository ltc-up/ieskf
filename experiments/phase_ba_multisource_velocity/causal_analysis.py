"""Native-time real intervention attribution, no reference fed to any run."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.track_evaluation import paired_errors
from ndt_global.validation import static_transform
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--ay-root',type=Path,required=True);a=p.parse_args();motion=json.loads(Path('data/phase_ao_ap_validation/loop44.json').read_text());fixed=np.load('data/phase_as_accuracy_quality/single_lap/evaluation.npz');corner=fixed['corner'];tan=fixed['tangent'];base=json.loads(Path('data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json').read_text());ay=json.loads((a.ay_root/'XY_lap/transactions.json').read_text());ns=np.array([r['ref'] for r in base]);out={};series={}
def stats(v):
 v=np.asarray(v);v=v[np.isfinite(v)];return dict(n=len(v),mean=float(np.mean(v)),RMS=float(np.sqrt(np.mean(v*v))),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),abs_max=float(np.max(abs(v)))) if len(v) else dict(n=0)
for name in ['F_lap','B_lap','J_lap','C_lap','W_lap']:
 tx=json.loads((a.root/name/'transactions.json').read_text());ev=np.load(a.root/name/'evaluation/evaluation.npz');prior=np.array([r['seed'] for r in tx])[:,:3,3];N=np.array([r['ndt']['final'] for r in tx])[:,:3,3];post=ev['candidate'];G=ev['G_0.075'];valid=ev['historical_valid_0.075'];whole={}
 for c in [1,3,5,8,9]:
  idx=np.flatnonzero(corner==c);lo=ns[idx[0]]-3_000_000_000;hi=ns[idx[-1]]+500_000_000;sel=(ns>=lo)&(ns<=hi);good=sel&valid;inside=(corner==c)&valid
  whole[str(c)]=dict(window_frames=int(sum(sel)),reference_frames=int(sum(good)),corner_reference={k:stats(abs(paired_errors(P,G,tan)[1][inside])) for k,P in [('prior',prior),('NDT',N),('posterior',post)]},prior_NDT_longitudinal=stats(paired_errors(prior,N,tan)[0][sel]),prior_NDT_lateral=stats(paired_errors(prior,N,tan)[1][sel]),observed_update_lateral=stats(paired_errors(post,prior,tan)[1][sel]))
 out[name]=dict(fixed_windows='original AS segmentation,3s before and.5s after; no output-selected window',corners=whole)
 series[name+'_prior_NDT_lateral']=paired_errors(prior,N,tan)[1];series[name+'_posterior_reference_lateral']=paired_errors(post,G,tan)[1];series[name+'_prior_reference_lateral']=paired_errors(prior,G,tan)[1]
 if name=='B_lap':
  last={}
  for line in (a.root/name/'optical_updates.jsonl').open():
   e=json.loads(line);last[e['message_id']]=e
  changed=sorted([e for e in last.values() if e['update'].get('accepted') and e['update'].get('frontend_used')=='BRAKE_FILTERED_X'],key=lambda r:r['header_ns']);first=changed[0]['header_ns'];prefix=[i for i,r in enumerate(ay) if r['complete']<first];err={}
  for k,fn in dict(seed=lambda r:r['seed'],P=lambda r:r['prior']['P'],NDT=lambda r:r['ndt']['final']).items():err[k]=max([float(np.max(abs(np.asarray(fn(tx[i]))-np.asarray(fn(ay[i]))))) for i in prefix],default=0.)
  out[name]['intervention']=dict(changed_optical_updates=len(changed),unchanged_completed_frames_before_first=len(prefix),pre_intervention_max_difference=err,interpretation='complete/availability boundary used: a Deskew acquisition can straddle native Header; no future/reference runtime input')
 if name=='J_lap':
  b=np.array([r['prior']['optical_bias'] for r in tx]);out[name]['estimated_optical_offset_m_s']={axis:stats(b[:,i]) for i,axis in enumerate(['sensor_x','sensor_y'])};out[name]['bias_qualification']='estimated nuisance state, not physical calibration';series['J_bias']=b
# Existing NDT pose difference is an interval-average diagnostic only.
T=np.array([r['ndt']['final'] for r in base]);dt=np.diff(ns)*1e-9;omega=np.array([Rotation.from_matrix(T[i,:3,:3].T@T[i+1,:3,:3]).as_rotvec()/dt[i] for i in range(len(dt))]);mid=(ns[:-1]+ns[1:])//2;v=[]
for i,h in enumerate(dt):
 Rm=T[i,:3,:3]@Rotation.from_rotvec(omega[i]*h/2).as_matrix();v.append(Rm.T@(T[i+1,:3,3]-T[i,:3,3])/h)
v=np.array(v);w=[e for e in motion['events'] if e['kind']=='wheel'];wt=np.array([e['header_ns'] for e in w]);wv=np.array([e['speed'] for e in w]);optic=json.loads((a.ay_root/'optical.json').read_text())['raw_topics']['/sensor/kistler/measurement'];ot=np.array([e['header_ns'] for e in optic]);ov=np.array([e['raw'] for e in optic]);Tr=static_transform(motion['transforms'],'base_link','kistler');op=(v+np.cross(omega,Tr[:3,3]))@Tr[:3,:3];ow=np.interp(mid-wt[0],wt-wt[0],wv);ox=np.interp(mid-ot[0],ot-ot[0],ov[:,0]);oy=np.interp(mid-ot[0],ot-ot[0],ov[:,1]);out['LiDAR_velocity_diagnostic']=dict(wheel_minus_pose_difference_X=stats(ow-v[:,0]),opticalX_minus_pose_difference_prediction=stats(ox-op[:,0]),opticalY_minus_pose_difference_prediction=stats(oy-op[:,1]),qualification='same map poses already fused into original15D; interval-average derivative noise/cross-frame correlation; never fused again')
series['native_header']=ns;series['corner']=corner;np.savez_compressed(a.root/'causal_series.npz',**series);(a.root/'causal_analysis.json').write_text(json.dumps(out,indent=2)+'\n')
