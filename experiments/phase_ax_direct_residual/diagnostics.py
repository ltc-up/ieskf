"""Aggregate real-run mechanism, geometry, retention and repeatability diagnostics."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.direct_iterated import augmented_chart
from geometry_ieskf.state import State,inject
from geometry_ieskf.common import jr,skew
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--motion',type=Path,required=True);p.add_argument('--corners',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
from ndt_global.validation import static_transform
import sys;sys.path.insert(0,str(Path(__file__).parents[1]/'phase_aw_native_gicp_fusion'));from evaluate import stats
read=lambda d:[json.loads(s) for s in (a.root/d/'transactions.jsonl').open()];lap=read('A_lap');inst=read('A_instrumented_lap');multi=read('A_multilap');corners=np.load(a.corners)['corner'];lever=static_transform(json.loads(a.motion.read_text())['transforms'],'base_link','imu_bosch')[:3,3];frozen=json.loads((a.baseline/'transactions.json').read_text());sig=[];gain=[];dlat=[];noise=[];cross=[];Perr=[];commonpos=[];commonrot=[];rpc=[];cost_up=0;refreshes=0
for r in inst:
 u=r['update'];s=u['prior_state'];x=State(*[np.array(s[k]) for k in ['p','v','R','bg','ba']],s['t']);z=np.r_[u['delta'],u['shared_nuisance']];y=inject(x,z[:15]);A=augmented_chart(y,lever,z);ix=[0,1,2,6,7,8];v=np.linalg.inv(A[:,:15][:,ix]);H15=np.array(u['final_H15']);H6=v.T@H15[np.ix_(ix,ix)]@v;H6=(H6+H6.T)/2;P=np.array(r['prior']['P']);Pi=np.zeros((21,21));Pi[:15,:15]=P;Pi[15:,15:]=np.diag([np.deg2rad(1)**2]*3+[.5**2]*3);Q=np.linalg.inv(np.linalg.inv(Pi)+A.T@H6@A);Perr.append(abs(Q[:15,:15]-np.array(u['posterior_chart'])).max());lat=x.R[:,1];B=np.zeros((3,15));B[:,:3]=np.eye(3);B[:,6:9]=y.R@skew(lever)@jr(z[6:9]);Rc=y.R@Rotation.from_rotvec(z[15:18]).as_matrix();e=np.r_[np.zeros(3),Rc.T@lat];gain.append(float(lat@B@Q[:15,:]@A.T@H6@e));Hbase=np.zeros((3,15));Hbase[:,:3]=np.eye(3);Hbase[:,6:9]=x.R@skew(lever);sig.append(float(np.sqrt(lat@Hbase@P@Hbase.T@lat)));delta=(y.p-y.R@lever)-(x.p-x.R@lever);dlat.append(float(lat@delta));noise.append(z[19]);commonpos.append(float(np.linalg.norm(z[18:])));commonrot.append(float(np.linalg.norm(z[15:18])));rpc.extend(u.get('rpc_timings',[]));its=u['iterations']
 for i,k in enumerate(its[:-1]):
  if k.get('frozen_trial_objective',k['objective'])>k['objective']+1e-8:cost_up+=1
  if i==0:refreshes+=its[i+1]['objective']>k.get('frozen_trial_objective',k['objective'])+1e-8
out=dict(repeat=dict(frames=len(inst),same_acceptance=all(r['update']['accepted']==s['update']['accepted'] for r,s in zip(lap,inst)),seed_max=max(abs(np.array(r['seed'])-np.array(s['seed'])).max() for r,s in zip(lap,inst)),priorP_max=max(abs(np.array(r['prior']['P'])-np.array(s['prior']['P'])).max() for r,s in zip(lap,inst)),delta_max=max(abs(np.array(r['update']['delta'])-np.array(s['update']['delta'])).max() for r,s in zip(lap,inst))),joint_reconstruction_max_error=float(max(Perr)),conditional_same_factor_unit_lateral_response=stats(gain),prior_body_lateral_sigma=stats(sig),actual_body_lateral_correction=stats(dlat),shared_nuisance=dict(translation_norm=stats(commonpos),rotation_norm_rad=stats(commonrot),body_lateral_lag1=float(np.corrcoef(noise[:-1],noise[1:])[0,1])),frozen_line_search_objective_increases=int(cost_up),outer_refresh_objective_increases=int(refreshes),RPC_wall_ms=stats([r['wall_ms'] for r in rpc]),service_ms=stats([r['service_ms'] for r in rpc if r['service_ms'] is not None]),corners={str(k):dict(samples=int(sum(corners[:len(inst)]==k)),prior_body_lateral_sigma=stats(np.array(sig)[corners[:len(inst)]==k]),unit_lateral_response=stats(np.array(gain)[corners[:len(inst)]==k]),actual_lateral_correction=stats(np.array(dlat)[corners[:len(inst)]==k]),shared_lateral_nuisance=stats(np.array(noise)[corners[:len(inst)]==k])) for k in range(10)})
# Native GPU Gaussian shape; never publish map payload.
dt=np.dtype([('key','<u8'),('n','<u4'),('a','<f4',(9,))]);g=np.fromfile(a.root/'map0105_gaussians.bin',dt,offset=8);v=g['a'];M=np.zeros((len(g),3,3));M[:,0,0]=v[:,3];M[:,0,1]=M[:,1,0]=v[:,4];M[:,0,2]=M[:,2,0]=v[:,5];M[:,1,1]=v[:,6];M[:,1,2]=M[:,2,1]=v[:,7];M[:,2,2]=v[:,8];e=np.linalg.eigvalsh(M);out['native_Gaussian_shape']=dict(count=len(g),normal_anisotropy=stats(e[:,2]/e[:,1]),eligible=int(sum(e[:,2]>10*e[:,1])),native_density_invariance='FAILED_AS_SHOWN_BY_SEPARATE_REAL_CUDA_SYNTHETIC_CONTROL')
# Preserve all attempted actual runs; no requested/processed conflation.
out['runs']={}
for name in ['A40_v1','A40_v2','A40_v3','A40_v4','A_lap','A_multilap','A_instrumented_lap','B40']:
 s=json.loads((a.root/(name+'_summary.json')).read_text());out['runs'][name]={k:s[k] for k in ['requested_frames','processed_frames','delayed_15D_updates','duration_s','stop_reason','decisions','wall_ms']}
a.output.write_text(json.dumps(out,indent=2)+'\n')
