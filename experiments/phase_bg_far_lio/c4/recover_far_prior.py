"""Qualified algebraic prior recovery, never an EKF re-run.
R_yaw above its floor identifies the actual used compensation delay. Both
compensations use the same pre-tick FIFO (native node logs before FIFO push).
Reject any FIFO identity mismatch rather than assuming debug is actual input.
"""
import json,argparse
from pathlib import Path
import numpy as np
B=None

def stats(a):
 a=np.asarray(a,float);return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(abs(a),95)),rmse=float(np.sqrt(np.mean(a*a))),max_abs=float(max(abs(a)))) if len(a) else dict(n=0)
def main(root,c1,refpath,evalpath):
 global B
 B=int(json.load(open(Path(root)/'input_time_contract.json'))['bag_origin_ns'])
 E=[];D=[]
 for line in open(Path(c1)/'far_recorded_init/records.jsonl'):
  r=json.loads(line)
  if r['kind']=='ekf':E.append(r)
  elif r['kind']=='timing_debug' and r['node']=='StateEstimation':D.append(r)
 assert len(E)==len(D)
 P=np.array([r['position'] for r in E]);t=np.array([r['stamp_ns'] for r in E]);ref=np.load(refpath);tt=ref['header'];tan=np.load(evalpath)['tangent'];out=[];mismatch=[];floor=0
 def shift(i,tau):
  j=int(tau/.01);f=tau/.01-j
  if j<0 or j+2>i:raise ValueError('FIFO_UNAVAILABLE')
  return P[i-1]-P[i-1-j]+f*(P[i-1-j]-P[i-2-j])
 for i,(e,d) in enumerate(zip(E,D)):
  ti=(int(e['stamp_ns'])-B)/1e9;g='normal8' if 307.306<=ti<=313.706 else 'right3' if 339.406<=ti<=343.806 else None
  if g is None:continue
  v=d['values'];ry=v.get('kalman_filter/lidar_gicp/R_psi_rad',0)
  if ry<=.0002000001:floor+=1;continue
  tau=(ry/.0001-1)/10;td=v['pose/lidar_gicp/virtual/msg_delay'];zdebug=np.array([v['pose/lidar_gicp/virtual/position_m/'+k] for k in 'xyz']);zm=np.array([v['pose/lidar_gicp/measured/position_m/'+k] for k in 'xyz'])
  fd=shift(i,td);fm=shift(i,v['pose/lidar_gicp/measured/msg_delay']);err=float(np.max(abs(zdebug-zm-(fd-fm))));mismatch.append(err)
  # Float32 debug fields round map-coordinate arithmetic; enforce a
  # coordinate-scaled forward rounding bound, retain strict sensitivity.
  bound=8*np.finfo(np.float32).eps*max(1.,float(np.max(abs(zdebug))),float(np.max(abs(zm))))
  if err>bound:continue
  zu=zdebug+shift(i,tau)-fd;res=np.array([v['kalman_filter/lidar_gicp/res_'+k+'_m'] for k in 'xyz']);prior=zu-res;post=P[i];ix=int(np.argmin(abs(tt-int(e['stamp_ns']))));n=np.r_[-tan[ix,1],tan[ix,0],0]
  row=dict(group=g,stamp_ns=int(e['stamp_ns']),index=i,used_delay_s=float(tau),debug_delay_s=td,fifo_identity_maxerr=err,receipt_pair_gap_ns=abs(e['received_wall_ns']-d['received_wall_ns']),joint_correction_lateral=float(n@(post-prior)),prior=prior.tolist(),post=post.tolist(),compensated_used=zu.tolist(),reference={})
  for key in ['0.0','0.075','0.15']:
   G=ref['G_'+key];gr=np.array([np.interp(int(e['stamp_ns']),tt,G[:,k]) for k in range(3)]);row['reference'][key]=dict(prior_lateral=float(n@(prior-gr)),post_lateral=float(n@(post-gr)),used_lidar_lateral=float(n@(zu-gr)))
  out.append(row)
 result=dict(scope='qualified delay-selected real C1 subset, algebraic reconstruction; joint update includes tilt/velocity, NOT isolated lidar K attribution',paired_records=len(E),above_floor_fifo_checks=stats(mismatch),floor_ineligible=floor,strict_10um_pass=sum(x<=1e-5 for x in mismatch),rounding_bound='8*eps_float32*max(1,abs(debug map coordinates)); no trajectory-error threshold',qualified=len(out),groups={})
 for g in ['normal8','right3']:
  q=[x for x in out if x['group']==g];result['groups'][g]={'n':len(q),'joint_lateral_correction':stats([x['joint_correction_lateral'] for x in q]),'reference':{}}
  for key in ['0.0','0.075','0.15']:result['groups'][g]['reference'][key]={k:stats([x['reference'][key][k] for x in q]) for k in ['prior_lateral','post_lateral','used_lidar_lateral']}
 Path(root,'analysis/far_recovered_prior.json').write_text(json.dumps(out,indent=2));Path(root,'analysis/far_recovery_aggregate.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument(n) for n in ['root','c1','refpath','evalpath']];a=p.parse_args();main(**vars(a))
