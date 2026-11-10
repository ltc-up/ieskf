"""Frozen native-time GNSS ENGINEERING reference, never filter input.
Reuses AW/AT reference conversion, masks, tangents and fixed0/75/150ms cases.
"""
import argparse,json,sys,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
sys.path.insert(0,str(Path(__file__).parents[1]/'phase_aw_native_gicp_fusion'))
from evaluate import reference,poses,stats
from ndt_global.track_evaluation import interpolate_reference,paired_errors
from ndt_global.validation import static_transform
p=argparse.ArgumentParser()
for k in ['run','baseline','motion','as-evaluation','at-bindings','reference-cache','run-summary','output','private']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--expected',type=int,required=True);a=p.parse_args();a.private.mkdir(exist_ok=True);tx=json.loads((a.run/'transactions.json').read_text());base=json.loads((a.baseline/'transactions.json').read_text())[:len(tx)];ns=np.array([r['ref'] for r in tx],np.int64);assert [r['ref'] for r in base]==ns.tolist();n=len(tx);motion=json.loads(a.motion.read_text());lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3];pre,A,RA=poses(tx,lever);_,F,RF=poses(base,lever);frozen=np.load(a.as_evaluation);corner=frozen['corner'][:n];tan=frozen['tangent'][:n];old=[e for e in motion['events'] if e['kind']=='pose'];origin=int(ns[0]);et=(np.array([e['header_ns'] for e in old],np.int64)-origin)*1e-9;t=(ns-origin)*1e-9;B,bok=interpolate_reference(et,np.array([e['position'] for e in old]),np.ones(len(old),bool),t,.1);RB=Slerp(et,Rotation.from_quat([e['quaternion_xyzw'] for e in old]))(t).as_matrix();refs=reference(json.loads(a.at_bindings.read_text()),ns,a.reference_cache);names={'AX_direct':(A,RA),'baseline_NDT_pose':(F,RF),'old_EKF':(B,RB)}
def metric(pos,rot,ref,mask):
 es,ed=paired_errors(pos,ref['G'],tan);dy=np.angle(np.exp(1j*(Rotation.from_matrix(rot).as_euler('xyz')[:,2]-Rotation.from_matrix(ref['R']).as_euler('xyz')[:,2])));return dict(XY=stats(np.linalg.norm((pos-ref['G'])[mask,:2],axis=1)),lateral=stats(abs(ed[mask])),signed_lateral=stats(ed[mask]),longitudinal=stats(abs(es[mask])),yaw_abs_deg=stats(abs(np.rad2deg(dy[mask]))),Z_difference=stats((pos-ref['G'])[mask,2]))
sensitivity={};private={}
for d,ref in refs.items():
 rows={}
 for convention,ok in [('historical_common',ref['historical_valid']&bok),('raw_extended',ref['raw_valid']&bok)]:
  rows[convention]=dict(valid=int(ok.sum()),coverage=float(ok.mean()),overall={k:metric(v,R,ref,ok) for k,(v,R) in names.items()},corners={str(c):{k:metric(v,R,ref,ok&(corner==c)) for k,(v,R) in names.items()} for c in sorted(set(corner)) if (ok&(corner==c)).any()})
 sensitivity[d]=rows;private['G_'+d]=ref['G'];private['historical_valid_'+d]=ref['historical_valid'];private['raw_valid_'+d]=ref['raw_valid']
states=json.loads((a.run/'states.json').read_text());P=np.array([s['P'] for s in states]);accepted=np.array([r['update']['accepted'] for r in tx]);meta=json.loads(a.run_summary.read_text());loops=[r['update']['iterations'] for r in tx if 'iterations' in r['update']];steps=[x for r in loops for x in r];gaps=np.diff(ns[accepted])*1e-9;dpos=A-pre;lateral_update=np.einsum('ni,ni->n',dpos,RA[:,:,1]);out=dict(processed=n,expected=a.expected,accepted=int(accepted.sum()),duration_s=float(t[-1]),complete=n==a.expected,decisions=dict(collections.Counter(r['update']['reason'] for r in tx)),sensitivity=sensitivity,full_P=dict(min_eigenvalue=float(np.linalg.eigvalsh(P)[:,0].min()),symmetry_error=float(abs(P-P.transpose(0,2,1)).max()),sigmaZ=stats(np.sqrt(P[:,2,2]))),state=dict(Z_span=float(np.ptp([s['p'][2] for s in states])),vz=stats([s['v'][2] for s in states]),bg_norm=stats([np.linalg.norm(s['bg']) for s in states]),ba_norm=stats([np.linalg.norm(s['ba']) for s in states])),update=dict(body_lateral=stats(lateral_update),iterations=stats([len(r) for r in loops]),Joseph_error_max=max((r['update'].get('Joseph_error',0) for r in tx),default=0),final_white_step=stats([r[-1]['white_step'] for r in loops if r]),factors=stats([r[-1]['factors'] for r in loops if r]),groups=stats([r[-1]['groups'] for r in loops if r]),single_covariance_commit=all(r['update'].get('covariance_updates',0)==int(r['update']['accepted']) for r in tx)),performance=dict(startup_ms=meta['startup_ms'],processing_ms=meta['wall_ms'],observation_age_ms=stats([(r['complete']-r['ref'])*1e-6 for r in tx]),over50ms=int(sum((r['complete']-max(r['available'],r['ref']))*1e-6>50 for r in tx))),max_accepted_gap_s=float(max(gaps,default=0)),GNSS_feedback=False,reference_qualification='CONDITIONAL engineering reference; no NEES or certified accuracy',convergence_scope='frozen associations after two refreshes; conditional GN MAP, not rematched objective stationarity')
out['per_lap_75ms_raw']=[]
for lo,hi in [(0,1468),(1468,2820),(2820,4134)]:
 ok=refs['0.075']['raw_valid']&bok&(np.arange(n)>=lo)&(np.arange(n)<hi)
 if ok.any():out['per_lap_75ms_raw'].append(dict(start_index=lo,valid=int(ok.sum()),**{k:metric(v,R,refs['0.075'],ok) for k,(v,R) in names.items()}))
a.output.write_text(json.dumps(out,indent=2)+'\n');np.savez_compressed(a.private/'evaluation.npz',header=ns,corner=corner,tangent=tan,candidate=A,prior=pre,baseline=F,old_EKF=B,**private)
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(8,9));ax.plot(A[:,0],A[:,1],label='AX direct');ax.plot(F[:,0],F[:,1],label='frozen NDT pose');ax.plot(B[:,0],B[:,1],label='old EKF');ax.axis('equal');ax.legend();fig.tight_layout();fig.savefig(a.private/'private_trajectory.png');plt.close(fig)
for c in [1,3,9]:
 fig,ax=plt.subplots(figsize=(9,3));mask=(corner==c)&refs['0.075']['historical_valid'];
 for k,(v,R) in names.items():ax.plot(t[mask],paired_errors(v,refs['0.075']['G'],tan)[1][mask],label=k)
 ax.legend();ax.set_ylabel('engineering lateral difference m');fig.tight_layout();fig.savefig(a.private/f'corner{c}.png');plt.close(fig)
