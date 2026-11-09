"""Existing AR/private runs versus audited engineering references; no fitting."""
import argparse,csv,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_closing,label
from ndt_global.track_evaluation import Polyline,paired_errors,interpolate_reference
from ndt_global.validation import static_transform

def summary(x):
 x=np.asarray(x,float);x=x[np.isfinite(x)]
 return dict(n=len(x),mean=float(x.mean()),rmse=float(np.sqrt(np.mean(x*x))),p50=float(np.median(x)),p95=float(np.percentile(x,95)),p99=float(np.percentile(x,99)),max=float(x.max()),min=float(x.min())) if len(x) else dict(n=0)
def metric(x,ref,tangent,yaw,refyaw,valid):
 e=x-ref;es,ed=paired_errors(x,ref,tangent);dy=np.arctan2(np.sin(yaw-refyaw),np.cos(yaw-refyaw))
 return dict(XY=summary(np.linalg.norm(e[valid,:2],axis=1)),X=summary(e[valid,0]),Y=summary(e[valid,1]),Z=summary(e[valid,2]),longitudinal=summary(es[valid]),signed_lateral=summary(ed[valid]),abs_lateral=summary(abs(ed[valid])),yaw_deg=summary(np.rad2deg(dy[valid])))
def inside(points,poly):
 x,y=np.asarray(points).T;out=np.zeros(len(x),bool)
 for a,b in zip(poly,np.roll(poly,-1,axis=0)):
  out^=((a[1]>y)!=(b[1]>y))&(x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1]+1e-300)+a[0])
 return out

def run(a):
 a.private.mkdir(parents=True,exist_ok=True);data=json.loads(a.input.read_text());states=json.loads((a.run/'states.json').read_text());tx=json.loads((a.run/'transactions.json').read_text());n=len(states);ns=np.array([v['time_ns'] for v in states],np.int64);origin=int(ns[0]);t=(ns-origin)*1e-9;P=np.array([v['p'] for v in states]);Rs=np.array([v['R'] for v in states]);lever=static_transform(data['transforms'],'base_link','imu_bosch')[:3,3];A=P-np.einsum('nij,j->ni',Rs,lever);ay=Rotation.from_matrix(Rs).as_euler('xyz')[:,2]
 ev=[e for e in data['events'] if e['kind']=='pose'];et=np.array([e['header_ns'] for e in ev],np.int64);B,bok=interpolate_reference((et-origin)*1e-9,np.array([e['position'] for e in ev]),np.ones(len(ev),bool),t,.1);by=np.interp(t,(et-origin)*1e-9,np.unwrap(Rotation.from_quat([e['quaternion_xyzw'] for e in ev]).as_euler('xyz')[:,2]))
 g=list(csv.DictReader(a.gnss.open()));gt=np.array([float(v['timestamp'])-origin*1e-9 for v in g]);gv=np.array([int(v['truth_valid']) and float(v['gps_bracket_dt_sec'])<=.6 and int(v['attitude_bracket_valid']) for v in g],bool);gp=np.array([[float(v['base_link_'+k]) for k in 'xyz'] for v in g]);gy=np.unwrap(Rotation.from_quat([[float(v['base_q'+k]) for k in 'xyzw'] for v in g]).as_euler('xyz')[:,2]);G,gok=interpolate_reference(gt,gp,gv,t,.100001);gyaw,_=interpolate_reference(gt,gy,gv,t,.100001);gok&=bok
 c=np.genfromtxt(a.center,delimiter=',',names=True);line=Polyline(np.c_[c['Xref'],c['Yref']]);anchor=np.where(gok[:,None],G[:,:2],B[:,:2]);z=line.project(anchor);s=z['s'];tan=z['tangent'];sgrid=np.arange(0,line.length,1.);ki=np.interp(sgrid,line.s[:-1],c['Kref'][:len(line.p)]);mask=abs(ki)>=.002;ext=np.r_[mask[-30:],mask,mask[:30]];mask=binary_closing(ext,np.ones(20,bool))[30:-30];ids,nc=label(mask);cid=0;corner_grid=np.zeros(len(sgrid),int)
 for k in range(1,nc+1):
  ii=np.flatnonzero(ids==k)
  if len(ii)>=20:cid+=1;corner_grid[ii]=cid
 corner=corner_grid[np.minimum(s.astype(int),len(sgrid)-1)];corners=[];_,ad=paired_errors(A,G,tan);_,bd=paired_errors(B,G,tan);as_,_=paired_errors(A,G,tan);bs,_=paired_errors(B,G,tan);esab,edab=paired_errors(A,B,tan)
 left=np.genfromtxt(a.left,delimiter=',',names=True);right=np.genfromtxt(a.right,delimiter=',',names=True);L=np.c_[left['X'],left['Y']];R=np.c_[right['X'],right['Y']];ll=Polyline(L);rr=Polyline(R)
 def margin(q):
  dl=ll.project(q)['distance'];dr=rr.project(q)['distance'];corridor=inside(q,L)^inside(q,R);return np.minimum(dl,dr)*np.where(corridor,1,-1),dl,dr
 ma,la,ra=margin(A[:,:2]);mb,lb,rb=margin(B[:,:2]);mg,lg,rg=margin(G[:,:2]);risk=dict(A_center_margin=summary(ma),B_center_margin=summary(mb),GNSS_center_margin=summary(mg[gok]),A_center_outside_count=int(sum(ma<0)),B_center_outside_count=int(sum(mb<0)),GNSS_center_outside_count=int(sum((mg<0)&gok)),actual_body_or_wheel_crossing='NOT_QUALIFIED_NO_HISTORICAL_OUTLINE_CERTIFICATE')
 for k in range(1,cid+1):
  m=(corner==k)&gok;pos=np.flatnonzero(corner_grid==k);thirds=[]
  for j in range(3):
   sub=m&(s>=sgrid[pos[0]]+j*len(pos)/3)&(s<sgrid[pos[0]]+(j+1)*len(pos)/3);thirds.append(dict(section=['entry','middle','exit'][j],A_abs_lateral=summary(abs(ad[sub])),B_abs_lateral=summary(abs(bd[sub]))))
  corners.append(dict(id=k,turn='left' if np.mean(ki[pos])>0 else 'right',A_abs_lateral=summary(abs(ad[m])),B_abs_lateral=summary(abs(bd[m])),A_center_margin=summary(ma[corner==k]),B_center_margin=summary(mb[corner==k]),sections=thirds))
 sensitivity={}
 for dt in [-.075,0.,.075]:
  Q,ok=interpolate_reference(gt,gp,gv,t+dt,.100001);ok&=bok;sensitivity[str(dt)]=dict(coverage=int(sum(ok)),A_XY=summary(np.linalg.norm((A-Q)[ok,:2],axis=1)),B_XY=summary(np.linalg.norm((B-Q)[ok,:2],axis=1)))
 out=dict(label=a.label,reference='historical75ms GPS1-derived engineering base_link reference; not certified truth',frames=n,reference_valid=int(sum(gok)),reference_coverage=float(gok.mean()),A_GNSS=metric(A,G,tan,ay,gyaw,gok),B_GNSS=metric(B,G,tan,by,gyaw,gok),A_B=metric(A,B,tan,ay,by,bok),corners=corners,boundary=risk,time_sensitivity=sensitivity,reference_height_datum_qualified=False,boundary_survey_map_version_qualified=False,track_length_m=line.length)
 np.savez(a.private/'evaluation.npz',time_ns=ns,t=t,A=A,B=B,G=G,valid=gok,s=s,corner=corner,ad=ad,bd=bd,as_=as_,bs=bs,esab=esab,edab=edab,ma=ma,mb=mb,mg=mg,tangent=tan,track_d_A=line.project(A[:,:2])['d'],track_d_B=line.project(B[:,:2])['d'])
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 race=np.genfromtxt(a.raceline,delimiter=',',names=True)
 fig,ax=plt.subplots(figsize=(8,10));ax.plot(L[:,0],L[:,1],'k-',lw=.7,label='launch left/right');ax.plot(R[:,0],R[:,1],'k-',lw=.7);ax.plot(race['X'],race['Y'],lw=.7,label='current planned raceline');ax.plot(A[:,0],A[:,1],label='AR15D');ax.plot(B[:,0],B[:,1],label='production');ax.plot(G[gok,0],G[gok,1],'.',ms=1,label='GPS1 reference75ms');ax.axis('equal');ax.legend();fig.tight_layout();fig.savefig(a.private/'track_overlay.png');plt.close(fig)
 fig,ax=plt.subplots(3,1,figsize=(12,8));ax[0].scatter(s[gok],ad[gok],s=2,label='15D');ax[0].scatter(s[gok],bd[gok],s=2,label='EKF');ax[0].legend();ax[0].set_ylabel('signed lateral ref difference m');ax[1].scatter(s[gok],as_[gok],s=2);ax[1].scatter(s[gok],bs[gok],s=2);ax[1].set_ylabel('longitudinal ref difference m');ax[2].scatter(s,ma,s=2);ax[2].scatter(s,mb,s=2);ax[2].axhline(0,color='r');ax[2].set_ylabel('center margin m');fig.tight_layout();fig.savefig(a.private/'progress_error_margins.png');plt.close(fig)
 for k in range(1,cid+1):
  m=corner==k
  if not m.any():continue
  fig,ax=plt.subplots(figsize=(7,7));ax.plot(L[:,0],L[:,1],'k');ax.plot(R[:,0],R[:,1],'k');ax.plot(A[m,0],A[m,1],label='15D');ax.plot(B[m,0],B[m,1],label='EKF');mm=m&gok;ax.plot(G[mm,0],G[mm,1],'.',label='GNSS engineering');q=A[m,:2];ax.set_xlim(q[:,0].min()-20,q[:,0].max()+20);ax.set_ylim(q[:,1].min()-20,q[:,1].max()+20);ax.legend();ax.set_aspect('equal');fig.savefig(a.private/f'corner_{k}.png');plt.close(fig)
 return out
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','run','gnss','center','left','right','raceline','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--label',default='AR_existing_lap');a=p.parse_args();a.output.write_text(json.dumps(run(a),indent=2)+'\n')
