"""Bounded frozen comparisons. No filter/seed/control output path exists."""
import json,sys,time,hashlib,resource,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from factor_diagnostics import *
REPO=Path(__file__).resolve().parents[2];sys.path[:0]=[str(REPO/'experiments/phase_bg_far_lio/c4'),str(REPO/'experiments/phase_be_e0_e4')]
from frozen_diagnostic import read_pcd
from runtime import read_cloud,write_cloud,Service,partition

def load(item):
 p=read_pcd(item['source']).astype(float) if item['format']=='pcd' else read_cloud(item['source']).astype(float)
 return partition(p)[0] if item.get('original_holdout_fit') else p
def main(root):
 root=Path(root);plan=json.load(open(root/'plan.json'));assert resource.getrusage(resource.RUSAGE_SELF).ru_maxrss<3*1024**2;out=(root/'results.jsonl').open('x');m=json.load(open(plan['IMU_Wheel']));I=sorted([e for e in m['events'] if e['kind']=='imu' and e.get('valid')],key=lambda e:e['header_ns']);W=sorted([e for e in m['events'] if e['kind']=='wheel' and e.get('valid')],key=lambda e:e['header_ns']);Rbi=np.array(plan['R_base_imu']);cache={};ndt=Service(plan['matcher'],root/'ndt.log');new=0
 def geometry(h):
  key=h['source']
  if key not in cache:
   z=time.perf_counter();p=load(h);C=covariances(p);v=Voxels(p,C);cache[key]=(p,C,v,(time.perf_counter()-z)*1000)
  return cache[key]
 def rel(a,b):return motion(a,b,I,W,Rbi)
 def relative_error(T,a,b):
  # Recorded map trajectory is OFFLINE only, never a motion seed or target builder.
  poses=[e for e in m['events'] if e['kind']=='pose' and e.get('valid') and a-30_000_000<=e['header_ns']<=b]
  pa=[e for e in poses if e['header_ns']<=a];pb=[e for e in poses if e['header_ns']<=b]
  if not pa or not pb:return dict(reference='UNAVAILABLE')
  def pose(e):
   A=np.eye(4);A[:3,:3]=Rotation.from_quat(e['quaternion_xyzw']).as_matrix();A[:3,3]=e['position'];return A
  gt=np.linalg.inv(pose(pa[-1]))@pose(pb[-1]);err=np.linalg.inv(gt)@T;rv=Rotation.from_matrix(err[:3,:3]).as_rotvec();return dict(reference='recorded map-filter engineering output, NOT independent truth; preceding sample, no interpolation',translation_error=err[:3,3].tolist(),yaw_error_deg=float(np.rad2deg(np.arctan2(err[1,0],err[0,0]))),rotation_error_deg=float(np.rad2deg(np.linalg.norm(rv))),reference_dt_s=(pb[-1]['header_ns']-pa[-1]['header_ns'])/1e9)
 try:
  for case in plan['cases']:
   start=time.perf_counter();history=[h for h in case['history'] if Path(h['source']).is_file()];row=dict(id=case['id'],group=case['group'],map=case['map_variant'],A=case['map_result'],source_contract=case['source_contract'],B=[],C=[],D=None,failures=[])
   try:
    p,C,v,prep=geometry(case);assert len(p)==case['map_result']['source_n'];row.update(source_n=len(p),source_cov_voxel_prepare_ms=prep)
    selected={};
    for nominal in plan['B_nominal_ms']:
     candidates=[h for h in history if abs((case['t']-h['t'])/1e6-nominal)<=5]
     if not candidates:row['B'].append(dict(nominal_ms=nominal,status='MISSING_IDENTITY_QUALIFIED_HISTORY'));continue
     h=min(candidates,key=lambda h:abs((case['t']-h['t'])/1e6-nominal));hp,hC,hv,hprep=geometry(h);T=rel(h['t'],case['t']);tick=time.perf_counter();e=evaluate(p,C,hv,T);ms=(time.perf_counter()-tick)*1000;e.update(nominal_ms=nominal,status='EQUATION_FACTOR_EVALUATED',dt_ms=(case['t']-h['t'])/1e6,target_identity=plan['identities'][h['source']]['sha256'],factor_ms=ms,target_prepare_ms=hprep,E_relative=T.tolist(),E_reference=relative_error(T,h['t'],case['t']));
     # Diagnostic only: evaluate existing map branch in the historical map anchor.
     old=h.get('final');old=np.loadtxt(old) if isinstance(old,str) and Path(old).exists() else np.asarray(old) if isinstance(old,list) else None
     if old is not None:
      Tmap=np.linalg.inv(old)@np.array(case['map_result']['final']);e['map_feedback_branch_control']=evaluate(p,C,hv,Tmap);e['map_control_is_not_independent_or_online']=True
     row['B'].append(e);selected[h['t']]=h
    # C: replay insertion/eviction using only earlier scans and E motion; no current map result.
    keys=[];management=0.;tick=time.perf_counter()
    for h in history:
     if not keys:keys.append(h);continue
     hp,_,_,_=geometry(h);union=np.zeros(len(hp),bool)
     for k in keys:
      _,_,kv,_=geometry(k);union|=kv.overlap(transform(rel(k['t'],h['t']),hp))
     if union.mean()>.7:continue
     keys.append(h)
     if len(keys)>3:
      _,_,nv,_=geometry(h);keys=[k for k in keys if k is h or geometry(k)[2].overlap(transform(rel(k['t'],h['t']),hp)).mean()>=.01]
     if len(keys)>3:
      scores=[]
      for k in keys[:-1]:
       kp,_,kv,_=geometry(k);latest=kv.overlap(transform(rel(k['t'],h['t']),hp)).mean();other=np.zeros(len(kp),bool)
       for j in keys[:-1]:
        if j is k:continue
        _,_,jv,_=geometry(j)
        D=rel(j['t'],k['t']) if j['t']<k['t'] else np.linalg.inv(rel(k['t'],j['t']));other|=jv.overlap(transform(D,kp))
       scores.append(latest*(1-other.mean()))
      keys.pop(int(np.argmin(scores)))
    management=(time.perf_counter()-tick)*1000;row['C_management_ms']=management
    for k in keys:
     _,_,kv,kprep=geometry(k);T=rel(k['t'],case['t']);tick=time.perf_counter();e=evaluate(p,C,kv,T);e.update(dt_ms=(case['t']-k['t'])/1e6,target_identity=plan['identities'][k['source']]['sha256'],factor_ms=(time.perf_counter()-tick)*1000,target_prepare_ms=kprep,E_reference=relative_error(T,k['t'],case['t']));row['C'].append(e)
    # D independent engineering comparator: explicitly concatenate up to3 past scans in earliest base, using E only.
    targets=history[-3:]
    if targets:
     anchor=targets[0]['t'];parts=[];tick=time.perf_counter()
     for h in targets:
      hp=geometry(h)[0];parts.append(hp if h['t']==anchor else transform(rel(anchor,h['t']),hp))
     target=np.vstack(parts);seed=rel(anchor,case['t']);write_cloud(root/'scratch_source.bin',p);write_cloud(root/'scratch_target.bin',target);prems=(time.perf_counter()-tick)*1000;sha=hashlib.sha256((root/'scratch_target.bin').read_bytes()).hexdigest();request=dict(case_id=case['id'],header=case['t'],group='D_local_target',scene='comparison_only',source_path=str(root/'scratch_source.bin'),target_path=str(root/'scratch_target.bin'),target_sha=sha,seed_common=seed.tolist(),roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,new+1],q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'));N,wall=ndt.call(request);new+=1;row['D']=dict(result=N,wall_ms=wall,target_assemble_write_ms=prems,n_history=len(targets),history_age_ms=[(case['t']-h['t'])/1e6 for h in targets],target_sha=sha,E_relative=seed.tolist(),E_reference=relative_error(seed,anchor,case['t']),candidate_reference=relative_error(np.array(N['final']),anchor,case['t']),map_not_used_for_target=True)
   except Exception as ex:row['failures'].append(repr(ex))
   row.update(comparison_wall_ms=(time.perf_counter()-start)*1000,RSS_peak_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss);out.write(json.dumps(row)+'\n');out.flush();print(case['id'],len(row['B']),len(row['C']),row['failures'],flush=True)
   if row['RSS_peak_KiB']>3*1024**2:raise RuntimeError('RSS_BUDGET')
   if sum(f.stat().st_size for f in root.rglob('*') if f.is_file())>256*1024**2:raise RuntimeError('STORAGE_BUDGET')
 finally:out.close();ndt.close();(root/'execution_status.json').write_text(json.dumps(dict(new_local_NDT_matches=new,filters_updated=0,frontend_seeds_changed=0,unique_geometry_cache=len(cache)),indent=2))
if __name__=='__main__':main(sys.argv[1])
