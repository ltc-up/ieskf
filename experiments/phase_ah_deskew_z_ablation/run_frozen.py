"""AH raw point comparison and frozen original-backend evaluations; no recursion."""
import argparse,json,subprocess,numpy as np
from pathlib import Path
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.geometry import GeometryFactor,GeometryCorrespondence
from geometry_ieskf.objective import state
from geometry_ieskf.state import difference,chart_map,inject
from geometry_ieskf.instrumented import InstrumentedUpdate
from geometry_ieskf.qualification import TrackedCorrespondence,local_optimality,pair_retention,eligible
from geometry_ieskf.partition import margins,POSE,OTHER
from geometry_ieskf.deskew_ablation import compensate,interpolate

def raw_fields(c):
 raw=Path(c['raw_path']).read_bytes();out={}
 for k in ['x','y','z','intensity','timestamp']:
  f=c['fields'][k];dtype=np.dtype({7:'f4',8:'f8'}[f['datatype']]).newbyteorder('>' if c['bigendian'] else '<');out[k]=np.ndarray((c['height'],c['width']),dtype=dtype,buffer=raw,offset=f['offset'],strides=(c['row_step'],c['point_step'])).ravel().astype(float)
 return out

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);ap.add_argument('--matcher',required=True);ap.add_argument('--only-groups',default='ABC');a=ap.parse_args();D=Path(a.data_root);O=Path(a.output);selection=json.load(open(O/'selection_private.json'))['selected'];contract=json.load(open(D/'history/phase_v/results/input_contract.json'));factor=GeometryFactor(contract['extrinsic']['lever_body_imu_m']);target=D/'inputs/map_target.bin';targetpoints=np.fromfile(target,'<f4',offset=8).reshape(-1,3).astype(float);pointstats=[];updates=[];jobs=[]
 for e in selection:
  h=e['job']['header'];T=np.array(e['frame']['T_native_from_common']);prior=state(e['event']['predicted']);P=np.array(e['event']['predicted']['P']);Knp=np.linalg.solve(P[np.ix_(POSE,POSE)],P[np.ix_(POSE,OTHER)]).T;groups={};native={};identities={}
  for g in ['A_original','A','B','C']:
   raw=np.fromfile(O/g/f'{h}_native.xyzi','<f4').reshape(-1,4);native[g]=raw;common=np.fromfile(O/g/f'{h}_A1.bin','<f4',offset=8).reshape(-1,3).astype(float);groups[g]=common@T[:3,:3].T+T[:3,3];identities[g]=dict(common_source_sha256=sha(O/g/f'{h}_A1.bin'),native_xyzi_sha256=sha(O/g/f'{h}_native.xyzi'))
  assert np.array_equal(native['A'],native['A_original']);assert np.array_equal(groups['A'],np.load(D/'history/phase_v'/e['frame']['source_file']))
  points=[];ts=[]
  for c in e['job']['clouds']:
   raw=raw_fields(c);X=np.array(c['T_body_lidar']);xyz=np.c_[raw['x'],raw['y'],raw['z']];points.append(xyz@X[:3,:3].T+X[:3,3]);ts.append((c['header_ns']-e['job']['native_reference_ns'])*1e-9+raw['timestamp'])
  points=np.concatenate(points);ts=np.concatenate(ts);poses=np.fromfile(e['job']['pose_path'],'<f8').reshape(-1,8);RR,pp=interpolate(ts,poses);ref=np.array(e['job']['T_odom_native']);refR=ref[:3,:3];refp=ref[:3,3];cpuchecks=[]
  for g in ['A','B','C']:
   cpu=compensate(points,RR,pp,refR,refp,factor.lever,g).astype(np.float32);error=abs(cpu-native[g][:,:3]);bound=64*np.finfo(np.float32).eps*(1+np.linalg.norm(points,axis=1)+np.linalg.norm(pp-refp,axis=1));assert np.all(error.max(1)<=bound);assert np.array_equal(native[g][:,3],native['A'][:,3]);cpuchecks.append(dict(group=g,max_abs_CPU_GPU_m=float(error.max()),max_fraction_conservative_roundoff_bound=float(np.max(error.max(1)/bound))))
  for g in ['B','C']:
   d=native[g][:,:3]-native['A'][:,:3];pointstats.append(dict(alias=e['alias'],group=g,roles=e['roles'],point_count=len(d),abs_xyz_quantiles_m=np.percentile(abs(d),[50,95,99,100],axis=0),abs_Z_quantiles_m=np.percentile(abs(d[:,2]),[50,95,99,100]),motion=e['motion'],CPU_GPU_checks=cpuchecks,input_identity=identities,raw_bounds_A=np.c_[native['A'][:,:3].min(0),native['A'][:,:3].max(0)]))
  for g in list(a.only_groups):
   source=groups[g];path=O/g/(e['alias']+'_native_source.bin');np.array([len(source)],'<u8').tofile(path)
   with path.open('ab') as f:np.asarray(source,'<f4').tofile(f)
   jobs.append(dict(case_id=e['alias']+'_'+g,header=e['event']['header_ns'],group=g,scene='AH_frozen',source_path=str(path.resolve()),target_path=str(target.resolve()),target_sha=sha(target),roi=e['case']['roi'],seed_common=factor.transform(prior),q=e['case']['q']))
   query=TrackedCorrespondence(targetpoints);solver=InstrumentedUpdate(factor);x,Q,info=solver.correct(prior,P,source,query);trace=solver.trace;z=np.array(info.get('delta',np.zeros(15)));props=trace['proposals'];last=np.array(props[-1]['absolute_frozen_delta']) if props else np.zeros(15);corr=query.get(x,source,factor);r,H,J=factor.linearize(x,corr['p'],corr['q'],corr['n']);w=corr['w']*solver.robust(r)[1];cert=local_optimality(P,z,H@chart_map(z),r,w);witness=query.get(inject(prior,z+cert['gn_delta']),source,factor);ret=pair_retention(corr,witness);cert.update(association_witness=ret['identical'],support_good=corr['good'],engineering_safe=min(margins(z))>=0);strict=eligible(cert,C['convergence_whitened_step']) if info['accepted'] else dict(accepted=False,status='REJECTED_KEEP_PRIOR',reasons=['ORIGINAL_SOLVER_REJECTED'])
   updates.append(dict(alias=e['alias'],group=g,prior=e['event']['predicted'],info=info,solver_trace=trace,old_accepted_delta=z,old_posterior=Q,raw_last_proposal=last,strict_qualification=strict,local_certificate=cert,association_probe=ret,K_np=Knp,raw_last_vz_contributions=Knp[2]*last[POSE],accepted_vz_contributions=Knp[2]*z[POSE],candidate_vz_conditional_error=float(last[5]-Knp[2]@last[POSE]),published=False,continuous_state_commits=0))
 if a.only_groups!='ABC':
  old=O/'definition_v1_IMU_origin_REJECTED';updates.extend(u for u in json.load(open(old/'frozen_updates_private.json')) if u['group'] not in a.only_groups);jobs.extend(j for j in json.load(open(old/'ndt_jobs.json')) if j['group'] not in a.only_groups);updates.sort(key=lambda u:(u['alias'],u['group']));jobs.sort(key=lambda j:j['case_id'])
 dump(O/'point_differences_private.json',pointstats);dump(O/'frozen_updates_private.json',updates);dump(O/'ndt_jobs.json',jobs)
 selectedjobs=[j for j in jobs if j['group'] in a.only_groups];dump(O/'ndt_partial_jobs.json',selectedjobs)
 with (O/'ndt.log').open('w') as f:subprocess.run([a.matcher,str(O/'ndt_partial_jobs.json'),str(O/'ndt_partial_results.jsonl')],stdout=f,stderr=subprocess.STDOUT,check=True)
 results=[json.loads(l) for l in open(O/'ndt_partial_results.jsonl')]
 if a.only_groups!='ABC':results.extend(j for j in map(json.loads,open(O/'definition_v1_IMU_origin_REJECTED/ndt_results.jsonl')) if j['group'] not in a.only_groups)
 results.sort(key=lambda j:j['case_id']);(O/'ndt_results.jsonl').write_text(''.join(json.dumps(j)+'\n' for j in results))
 print('AH raw point/IESKF/NDT runs',len(pointstats),len(updates),len(jobs))
if __name__=='__main__':main()
