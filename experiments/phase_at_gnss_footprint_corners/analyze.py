"""Frozen-run AT reference reconstruction and native-time causal decomposition.

All paths are explicit private inputs. No alignment fit, delay optimization, estimator
update, or synthetic vehicle outline. Historical delay hypotheses are fixed a priori.
"""
import argparse,csv,json,subprocess,hashlib
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation,Slerp
from scipy.spatial import ConvexHull
from ndt_global.validation import static_transform
from ndt_global.track_evaluation import interpolate_reference,Polyline,paired_errors
from ndt_global.footprint_audit import antenna_to_base,polygon_audit,project_footprint

def stats(x):
 x=np.asarray(x);x=x[np.isfinite(x)]
 return dict(n=len(x),mean=float(x.mean()),rmse=float(np.sqrt(np.mean(x*x))),p50=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(x.max()),min=float(x.min())) if len(x) else dict(n=0)
def run(a):
 a.private.mkdir(parents=True,exist_ok=True);raw=json.loads(a.raw.read_text());motion=json.loads(a.motion.read_text());tx=json.loads((a.run/'transactions.json').read_text());old=np.load(a.as_npz);csvrows=list(csv.DictReader(a.gnss.open()));origin=tx[0]['ref']*1e-9;t=np.array([x['ref'] for x in tx])*1e-9-origin
 cfg=yaml.safe_load(a.config.read_text());cfg=cfg.get('/**',{}).get('ros__parameters',cfg)
 T=static_transform(raw['/tf_static'],'base_link','vectornav_ant1');lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3]
 attitude=raw['/vectornav/raw/attitude'];at=np.array([x['header']-origin for x in attitude]);q=np.array([x['q'] for x in attitude]);keep=np.r_[True,np.diff(at)>0];at=at[keep];q=q[keep]
 rr=Rotation.from_euler('xyz',cfg['pose_orientation_bias'])*Rotation.from_quat([2**-.5,2**-.5,0,0])*Rotation.from_quat(q)*Rotation.from_quat([1,0,0,0])*Rotation.from_matrix(T[:3,:3]).inv();spl=Slerp(at,rr)
 def orientations(query):return spl(np.clip(query,at[0],at[-1])).as_matrix()
 gps=[x for x in raw['/vectornav/raw/gps'] if x['fix']==8];gt=np.array([x['header']-origin for x in gps]);ll=np.array([x['lla'] for x in gps]);payload=''.join(' '.join(map(str,x))+'\n' for x in ll)
 enu=np.loadtxt(subprocess.run(['CartConvert','-l',*map(str,cfg['reference_point']),'-p','10'],input=payload,text=True,capture_output=True,check=True).stdout.splitlines())
 def rebuild(query,delay):
  Q,ok=interpolate_reference(gt-delay,enu,np.ones(len(gt),bool),query,.600001);j=np.searchsorted(at,query);jj=np.clip(j,1,len(at)-1);ok&=(j>0)&(j<len(at))&(at[jj]-at[jj-1]<=.05)
  return antenna_to_base(Q,orientations(query),T[:3,3]),ok
 ct=np.array([float(x['timestamp'])-origin for x in csvrows]);cp=np.array([[float(x['base_link_'+k]) for k in 'xyz'] for x in csvrows]);cv=np.array([int(x['truth_valid']) and int(x['attitude_bracket_valid']) for x in csvrows],bool)
 G,gok=interpolate_reference(ct,cp,cv,t,.100001);priorR=np.array([x['prior']['R'] for x in tx]);priorp=np.array([x['prior']['p'] for x in tx]);delta=np.array([x['update']['delta'] for x in tx]);postR=priorR@Rotation.from_rotvec(delta[:,6:9]).as_matrix();prior=priorp-np.einsum('nij,j->ni',priorR,lever);post=priorp+delta[:,:3]-np.einsum('nij,j->ni',postR,lever);N=np.array([x['ndt']['final'] for x in tx])[:,:3,3]
 ev=[e for e in motion['events'] if e['kind']=='pose'];et=np.array([e['header_ns']*1e-9-origin for e in ev]);B,bok=interpolate_reference(et,np.array([e['position'] for e in ev]),np.ones(len(et),bool),t,.1);gok&=bok
 tan=old['tangent'];corner=old['corner'];s=old['s'];normal=np.c_[-tan[:,1],tan[:,0]]
 def errors(P,Q,ok):
  es,ed=paired_errors(P,Q,tan);return dict(XY=stats(np.linalg.norm(P[ok,:2]-Q[ok,:2],axis=1)),longitudinal=stats(es[ok]),lateral=stats(ed[ok]),abs_lateral=stats(abs(ed[ok])))
 sensitivity={}
 refs={}
 for delay in [0.,.075,.15]:
  A,ok=interpolate_reference(ct,cp,cv,t+delay-.075,.100001);ok&=bok
  R,rok=rebuild(t,delay);rok&=bok;refs[delay]=(R,rok)
  sensitivity[str(delay)]=dict(A_curve_query_shift=dict(A=errors(post,A,ok),B=errors(B,A,ok)),B_raw_gps_delay=dict(A=errors(post,R,rok),B=errors(B,R,rok)),A_vs_B_XY=stats(np.linalg.norm(A[ok&rok,:2]-R[ok&rok,:2],axis=1)))
 rebuilt,rv=rebuild(ct,.075);check=cv&rv;histq=Rotation.from_quat([[float(x['base_q'+k]) for k in 'xyzw'] for x in csvrows]);qr=Rotation.from_matrix(orientations(ct));qerr=(qr.inv()*histq).magnitude();equiv=stats(np.linalg.norm(rebuilt[check]-cp[check],axis=1))
 corners=[];private={};R,rok=refs[.075];valid=rok&gok
 K=np.array([x['update']['K'] for x in tx]);inn=np.array([x['update']['innovation'] for x in tx]);linear=np.einsum('nij,nj->ni',K,inn);gain_error=stats(np.max(abs(linear-delta),axis=1))
 for k in [1,3,9]:
  m=(corner==k)&valid;ed={name:paired_errors(P,R,tan)[1] for name,P in [('prior',prior),('ndt',N),('posterior',post),('production',B)]};idx=np.flatnonzero(m);parts=[]
  # Fixed AS entry/middle/exit thirds of the saved section progress interval.
  sk=s[corner==k];lo,hi=sk.min(),sk.max()
  for j in range(3):
   mm=m&(s>=lo+(hi-lo)*j/3)&(s<=lo+(hi-lo)*(j+1)/3);parts.append(dict(section=['entry','middle','exit'][j],**{n:stats(abs(e[mm])) for n,e in ed.items()}))
  corners.append(dict(id=k,reference_valid=int(sum(m)),error={n:stats(abs(e[m])) for n,e in ed.items()},update_increases_abs_reference_error=int(sum(abs(ed['posterior'][m])>abs(ed['prior'][m]))),lateral_update=stats(np.einsum('ni,ni->n',(post-prior)[:,:2],normal)[m]),parts=parts))
  private[str(k)]={n:e.tolist() for n,e in ed.items()}
 # Full polygon engine executes only on supplied authentic private outlines.
 def loadring(path):
  z=np.genfromtxt(path,delimiter=',',names=True);return np.c_[z['X'],z['Y']]
 L=loadring(a.left);Rt=loadring(a.right);lline=Polyline(L);rline=Polyline(Rt);boundary={};outlines={}
 front=static_transform(motion['transforms'],'base_link','front_wheel')[:3,3]
 BR=Rotation.from_quat([e['quaternion_xyzw'] for e in ev]);bspl=Slerp(et,BR);bR=bspl(np.clip(t,et[0],et[-1])).as_matrix()
 for name,P,RR in [('new15D',post,postR),('production',B,bR),('GNSS',R,orientations(t))]:
  f=P+np.einsum('nij,j->ni',RR,front);d=np.minimum(lline.project(P[:,:2])['distance'],rline.project(P[:,:2])['distance']);df=np.minimum(lline.project(f[:,:2])['distance'],rline.project(f[:,:2])['distance']);mm=valid if name=='GNSS' else np.ones(len(t),bool);boundary[name]=dict(rear_point_unsigned_distance=stats(d[mm]),front_axle_point_unsigned_distance=stats(df[mm]),body='NOT_VALIDATED_NO_HISTORICAL_OUTLINE',tyres='NOT_VALIDATED_NO_CONTACT_GEOMETRY')
  if a.geometry:
   geom=json.loads(a.geometry.read_text());outer,inner=(L,Rt) if abs(np.cross(L,np.roll(L,-1,axis=0)).sum())>abs(np.cross(Rt,np.roll(Rt,-1,axis=0)).sum()) else (Rt,L)
   for kind,polys in geom['polygons'].items():
    records=[];frame_flags=[]
    for i in np.flatnonzero(mm):
     frame_records=[]
     for poly in polys:
      wxy=project_footprint(poly,RR[i],P[i]);v=polygon_audit(wxy,outer,inner);records.append(v);frame_records.append(v)
     frame_flags.append(any(not v['contained'] for v in frame_records))
    imin=int(np.argmin([v['boundary_separation'] for v in records]));fi=int(np.flatnonzero(mm)[imin//len(polys)]);(a.private/(name+'_'+kind+'_polygon_records.json')).write_text(json.dumps(dict(frame_indices=np.flatnonzero(mm).tolist(),records=records)))
    outlines[name+'_'+kind]=dict(minimum_frame_index=fi,minimum_corner_id=int(corner[fi]),tested=len(records),not_contained=sum(not v['contained'] for v in records),geometry_qualification=geom['qualification'],boundary_separation_m=stats([v['boundary_separation'] for v in records]),boundary_intersections=sum(v['boundary_intersection'] for v in records),suspected_event_frames=int(sum(frame_flags)))
 # Observation decomposition uses the full gain; does not remove any cross terms.
 feedback={}
 for k in [1,3,9]:
  m=(corner==k)&valid
  pospart=np.einsum('nij,nj->ni',K[:,:3,:3],inn[:,:3]);rotpart=np.einsum('nij,nj->ni',K[:,:3,3:],inn[:,3:])
  feedback[str(k)]=dict(position_residual_to_imu_lateral_m=stats(np.einsum('ni,ni->n',pospart[:,:2],normal)[m]),attitude_residual_to_imu_lateral_m=stats(np.einsum('ni,ni->n',rotpart[:,:2],normal)[m]),gyro_bias_update_norm=stats(np.linalg.norm(delta[m,9:12],axis=1)),accel_bias_update_norm=stats(np.linalg.norm(delta[m,12:15],axis=1)),P_position_attitude_cross_frobenius=stats([np.linalg.norm(np.asarray(x['prior']['P'])[:3,6:9]) for i,x in enumerate(tx) if m[i]]))
 timing={}
 for topic,rows in raw.items():
  if topic=='/tf_static':continue
  hh=np.array([x['header'] for x in rows]);receipt=np.array([x['receipt'] for x in rows]);timing[topic]=dict(count=len(rows),header_to_record_s=stats(receipt-hh),header_gap_s=stats(np.diff(hh)),fix_counts={str(f):sum(x.get('fix')==f for x in rows) for f in sorted(set(x['fix'] for x in rows if 'fix'in x))})
 out=dict(frames=len(t),reference_label='engineering GNSS, not calibrated truth',historical_rebuild_3D_difference=equiv,historical_rebuild_rotation_rad=stats(qerr[check]),gain_delta_identity= gain_error,time_sensitivity=sensitivity,corners=corners,boundary=boundary,polygon_results=outlines,timing=timing,feedback=feedback,height_reference_difference_m=stats((post-R)[valid,2]),height_reference_after_documented_origin_change_m=stats((post-R)[valid,2]+float(cfg['reference_point'][2])*-1),calibrated_delay=False,height_datum='NOT_VALIDATED',physical_outline='NOT_VALIDATED',safety='NOT_VALIDATED')
 np.savez(a.private/'causal.npz',t=t,s=s,corner=corner,valid=valid,prior=prior,ndt=N,post=post,B=B,G=R,delta=delta,innovation=inn,K=K,P=np.array([x['prior']['P'] for x in tx]),R=np.array([x['update']['R'] for x in tx]))
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(figsize=(8,10));ax.plot(L[:,0],L[:,1],'k');ax.plot(Rt[:,0],Rt[:,1],'k');ax.plot(post[:,0],post[:,1],label='15D rear axle');ax.plot(B[:,0],B[:,1],label='production rear axle');ax.plot(R[valid,0],R[valid,1],'.',ms=2,label='raw-rebuilt GNSS rear axle');ax.axis('equal');ax.legend();fig.savefig(a.private/'rear_axle_overlay.png');plt.close(fig)
 for k in [1,3,9]:
  m=corner==k;fig,axs=plt.subplots(2,1,figsize=(10,8));
  for name,PP in [('prior',prior),('NDT',N),('posterior',post),('production',B)]:
   e=paired_errors(PP,R,tan)[1];axs[0].plot(t[m&valid],e[m&valid],label=name);axs[1].plot(PP[m,0],PP[m,1],label=name)
  axs[0].set_ylabel('signed lateral GNSS-reference difference m');axs[0].legend();axs[1].plot(R[m&valid,0],R[m&valid,1],'.',label='GNSS');axs[1].axis('equal');fig.savefig(a.private/f'corner_{k}_decomposition.png');plt.close(fig)
 fig,ax=plt.subplots();
 for kind in ['A_curve_query_shift','B_raw_gps_delay']:
  for name in ['A','B']:ax.plot([0,75,150],[sensitivity[str(v)][kind][name]['XY']['rmse'] for v in [0.,.075,.15]],'o-',label=kind+':'+name)
 ax.set_xlabel('declared GPS delay hypothesis ms');ax.set_ylabel('GNSS-reference XY RMSE m');ax.legend(fontsize=7);fig.savefig(a.private/'time_sensitivity.png');plt.close(fig)
 # Private plots use caller-supplied visual polygons only, never fabricated tyres.
 if a.geometry:
  geom=json.loads(a.geometry.read_text());poly=np.asarray(next(iter(geom['polygons'].values()))[0]);poly=poly if poly.shape[1]==3 else np.c_[poly,np.zeros(len(poly))]
  for k in [3,9]:
   fig,ax=plt.subplots(figsize=(9,8));ax.plot(L[:,0],L[:,1],'k');ax.plot(Rt[:,0],Rt[:,1],'k')
   ids=np.flatnonzero(corner==k)
   for name,PP,RR,color in [('15D',post,postR,'C0'),('production',B,bR,'C1')]:
    ax.plot(PP[ids,0],PP[ids,1],color=color,label=name)
    for i in ids[::max(1,len(ids)//20)]:
     w=PP[i]+poly@RR[i].T;w=w[ConvexHull(w[:,:2]).vertices];w=np.vstack([w,w[0]]);ax.plot(w[:,0],w[:,1],color=color,alpha=.4,lw=.6)
   pts=post[ids,:2];ax.set_xlim(pts[:,0].min()-10,pts[:,0].max()+10);ax.set_ylim(pts[:,1].min()-10,pts[:,1].max()+10);ax.set_aspect('equal');ax.legend();ax.set_title('Historical visual box assumption; tyre geometry unavailable');fig.savefig(a.private/f'corner_{k}_visual_polygon.png');plt.close(fig)
 a.output.write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for name in ['raw','motion','run','as_npz','gnss','config','private','output','left','right']:p.add_argument('--'+name.replace('_','-'),type=Path,required=True)
 p.add_argument('--geometry',type=Path);run(p.parse_args())
