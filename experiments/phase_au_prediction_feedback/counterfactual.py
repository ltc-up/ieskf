"""Frozen-state PSD sensitivities and short fixed-observation interventions.

No GNSS scoring, parameter selection, actual CUDA rematch or accuracy claim.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.ar_replay import AuditedPoseReplay
from ndt_global.feedback_diagnostics import audit_pose_update,base_position,skip_information_step
from recover_history import pack,unpack

def run(a):
 eq=json.loads(a.equivalence.read_text());assert eq['equivalent'];windows=json.loads(a.windows.read_text());events=json.loads((a.private/'canonical_events.json').read_text());arc=np.load(a.private/'canonical_checkpoints.npz');ck={k:arc[k] for k in arc.files};arc.close();ini=np.load(a.private/'initial.npz');lever=ini['lever'];representatives={w['section']:w['representative_frame'] for w in windows};matrix=[];branches=[]
 for section,frame in representatives.items():
  start=next(i for i,e in enumerate(events) if e['kind']=='ndt' and e['message_id']==10_000_000+frame);initial=(unpack(ck['before'][start],ck['t0'][start]),ck['P0'][start].copy(),dict(header_ns=int(ck['imu_header0'][start]),omega=ck['imu_omega0'][start].copy(),force=ck['imu_force0'][start].copy()));target=events[start];T=np.asarray(target['pose']);R=np.asarray(target['R']);axis=initial[0].R[:,1];end_ns=target['header_ns']+250_000_000;stop=start
  while stop<len(events) and events[stop]['header_ns']<=end_ns:stop+=1
  wheel=next(i for i in range(start+1,stop) if events[i]['kind']=='wheel')
  # Fixed prior/measurement sensitivities are matrices only; never fed to the run.
  for label,ps,rs in [('original',1.,1.),('P_half',.5,1.),('P_double',2.,1.),('R_half',1.,.5),('R_double',1.,2.)]:
   v=audit_pose_update(initial[0],ps*initial[1],T,rs*R,lever);H=v['H'];e=np.r_[axis,np.zeros(3)];d=v['delta'];Rafter=initial[0].R@Rotation.from_rotvec(d[6:9]).as_matrix();base_delta=d[:3]-(Rafter-initial[0].R)@lever;matrix.append(dict(section=section,frame=frame,case=label,P_scale=ps,R_scale=rs,unit_lateral_gain=float(e@H@v['K']@e),lateral_delta_m=float(axis@base_delta),delta_norm=float(np.linalg.norm(d)),minimum_posterior_eigenvalue=float(np.linalg.eigvalsh(v['P_post'])[0]),interpretation='fixed-state PSD sensitivity; not calibrated covariance or GNSS optimization'))
  data={}
  for label,skip in [('baseline',None),('skip_ndt_once',start),('skip_wheel_once',wheel)]:
   x,P,im=initial;checkpoint=(x.copy(),P.copy(),dict(im));replay=AuditedPoseReplay(*checkpoint,lever);observer=ExternalPose(lever,('x','y','z','rx','ry','rz'));records=[];ndt=[];maxmean=0.;maxcov=0.
   for i in range(start,stop):
    desc=events[i];e=dict(desc)
    if e['kind']=='ndt':
     e['observer']=observer;e['pose']=np.asarray(e['pose']);e['R']=np.asarray(e['R']);x,P,_=checkpoint;seed=np.eye(4);seed[:3,:3]=x.R;seed[:3,3]=x.p-x.R@lever;old=unpack(ck['before'][i],ck['t0'][i]);angle=(Rotation.from_matrix(old.R).inv()*Rotation.from_matrix(x.R)).magnitude();ndt.append(dict(event=i,frame=e['message_id']-10_000_000,seed_base=seed.tolist(),seed_change_m=float(np.linalg.norm(seed[:3,3]-(old.p-old.R@lever))),seed_change_rad=float(angle),frozen_final_pose=e['pose'].tolist(),rematched=False,skipped=i==skip))
    before=checkpoint;checkpoint=skip_information_step(replay,checkpoint,e) if i==skip else replay.step(checkpoint,e);x,P,im=checkpoint;accepted=int(e.get('last_update',{}).get('accepted',False)) if e['kind'] in ('ndt','wheel') else -1
    if e['kind']=='ndt':ndt[-1].update(accepted=bool(accepted),reason='EXPLICIT_SKIP' if i==skip else e.get('last_update',{}).get('reason','UNKNOWN'))
    records.append(dict(event=i,time=x.t,before=pack(before[0]),after=pack(x),P0=before[1],P1=P,accepted=accepted))
    if label=='baseline':maxmean=max(maxmean,float(np.max(abs(pack(x)-ck['after'][i]))));maxcov=max(maxcov,float(np.max(abs(P-ck['P1'][i]))))
   stem=a.private/f'counter_section{section}_{label}';np.savez_compressed(str(stem)+'.npz',**{k:np.asarray([v[k] for v in records]) for k in records[0]});Path(str(stem)+'_ndt.json').write_text(json.dumps(ndt));data[label]=dict(records=records,ndt=ndt)
   if label=='baseline':assert maxmean<=1e-10 and maxcov<=1e-10;branches.append(dict(section=section,case=label,frame=frame,events=stop-start,baseline_max_mean_difference=maxmean,baseline_max_P_difference=maxcov,actual_processed_span_s=(records[-1]['time']-target['header_ns'])*1e-9))
  reference=data['baseline']['records']
  for label in ['skip_ndt_once','skip_wheel_once']:
   records=data[label]['records'];changes=[]
   for base,changed in zip(reference,records):
    dbase=base_position(changed['after'],lever)-base_position(base['after'],lever);changes.append(float(axis@dbase))
   immediate_index=0 if label=='skip_ndt_once' else wheel-start;next_prior=[]
   for i,(b,c) in enumerate(zip(reference,records)):
    if events[b['event']]['kind']=='ndt' and b['event']> (start if label=='skip_ndt_once' else wheel):
     dbase=base_position(c['before'],lever)-base_position(b['before'],lever);next_prior.append(float(axis@dbase))
   immediate=changes[immediate_index];next_change=next_prior[0] if next_prior else None;last=records[-1]['after'];baseline_last=reference[-1]['after'];future=[v for v in data[label]['ndt'] if v['event']> (start if label=='skip_ndt_once' else wheel)]
   branches.append(dict(section=section,case=label,frame=frame,intervention_event_offset=immediate_index,events=stop-start,window_s=.25,actual_processed_span_s=(records[-1]['time']-target['header_ns'])*1e-9,immediate_fixed_axis_lateral_difference_m=immediate,next_prior_lateral_difference_m=next_change,next_prior_over_immediate_ratio=(next_change/immediate if next_change is not None and abs(immediate)>1e-8 else None),final_lateral_difference_m=changes[-1],max_future_seed_change_m=max((v['seed_change_m'] for v in future),default=0),max_future_seed_change_rad=max((v['seed_change_rad'] for v in future),default=0),final_velocity_difference_norm_m_s=float(np.linalg.norm(last[3:6]-baseline_last[3:6])),final_bg_difference_norm=float(np.linalg.norm(last[15:18]-baseline_last[15:18])),final_ba_difference_norm=float(np.linalg.norm(last[18:21]-baseline_last[18:21])),future_NDT_final_result_changed=False,future_NDT_accepted=sum(v['accepted'] for v in future),future_NDT_rejected=sum(not v['accepted'] for v in future),rematched=False,interpretation='fixed-observation conditional intervention; own seed changes are logged, actual matcher response unknown'))
   import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
   fig,ax=plt.subplots();ax.plot((np.array([r['time'] for r in records])-target['header_ns'])*1e-9,changes);ax.set_xlabel('seconds after median-frame intervention');ax.set_ylabel('fixed-axis base lateral difference m');ax.set_title(label+'; frozen NDT, no rematch');fig.tight_layout();fig.savefig(a.private/f'counter_section{section}_{label}.png');plt.close(fig)
 a.output.write_text(json.dumps(dict(matrix_sensitivities=matrix,short_window_branches=branches,GNSS_scored=False,true_closed_loop=False,original_parameters_changed=False,fixed_observations=True),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['private','windows','equivalence','output']:p.add_argument('--'+k,type=Path,required=True)
 run(p.parse_args())
