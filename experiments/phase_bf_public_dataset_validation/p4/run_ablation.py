"""Bounded 20+2+(conditional40)+10 source-only experiments, at most72.
Uses qualified P3 Fitness OFF binary. No filter, reference or R construction.
"""
import argparse,hashlib,json,os,struct,subprocess,time
from pathlib import Path
import numpy as np
from sampling import COUNTS,SPACE_SEEDS,RANDOM_SEED,space_order,random_order,quality
FRAMES=(60,64,65,66,76)
BASE='3c1c45e7f3accb7a29008642c192b84d88e2675d'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def read(p):
 with Path(p).open('rb') as f:n=struct.unpack('<Q',f.read(8))[0];a=np.fromfile(f,dtype='<f4').reshape(-1,3)
 assert n==len(a) and np.isfinite(a).all();return a

def main(a):
 root=Path(a.root);bd=Path(a.bd);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
 assert os.statvfs(out).f_bavail*os.statvfs(out).f_frsize>1<<30
 binary=root/'p3/pairs_01/matcher_fitness_toggle';lib=bd/'build/p0/libndt_gpu_matcher.so';assert sha(binary)=='7baf2be005c00db07d1963a21fc16c4b9d14c2a89ef18a1ff34dce8ea38adcff' and sha(lib)=='0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c'
 p2=root/'p2/five_01/execution_receipt.json';old={x['frame']:x for x in json.loads(p2.read_text())['records']};assert all(f in old for f in FRAMES)
 p2ids=json.loads((root/'p2/five_01/identity_after.json').read_text());sources={};inputs=[binary,lib,p2,root/'p3/pairs_01/execution_receipt.json'];prep={}
 for f in FRAMES:
  req=old[f]['request'];assert req['max_iterations']==35 and req['q']==50 and req['bd_config']==dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')
  for name in ['source_path','target_path']:assert sha(req[name])==p2ids['files'][req[name]];inputs.append(Path(req[name]))
  t=time.perf_counter_ns();sources[f]=read(req['source_path']);prep[str(f)]=(time.perf_counter_ns()-t)*1e-6;assert len(sources[f])==old[f]['result']['source_n']
 before={str(p):sha(p) for p in inputs};preregister=dict(baseline=BASE,frames=FRAMES,counts=COUNTS,space_seeds=SPACE_SEEDS,random_seed=RANDOM_SEED,actual_seed_rule='base_seed + frame',space_method='36azimuth x 5horizontal-radius x 6z strata; 1m voxel round robin inside strata; fixed random tie order; force six xyz extrema; exact nested prefixes',stage_B_trigger='any frame Stage A maximum pairwise final XY separation >0.25m OR convergence flags differ; no reference used',maximum_calls=72,stageA=20,full_off_supplements=2,stageB=40,stageC=10,fitness='OFF',trace=False,raw_source_read_ms=prep,inputs=before)
 save(out/'preregistration.json',preregister)
 orders={};order_times={};basequality={};indices={};sampling=[]
 for f,points in sources.items():
  basequality[f]=quality(points)
  for seed in SPACE_SEEDS:
   t=time.perf_counter_ns();o=space_order(points,seed+f);elapsed=(time.perf_counter_ns()-t)*1e-6;assert len(o)==len(points) and len(np.unique(o))==len(points)
   orders[(f,'space',seed)]=o;order_times[(f,'space',seed)]=elapsed;indices[f'{f}_space_{seed}']=o[:20000]
  t=time.perf_counter_ns();o=random_order(points,RANDOM_SEED+f);elapsed=(time.perf_counter_ns()-t)*1e-6;orders[(f,'random',RANDOM_SEED)]=o;order_times[(f,'random',RANDOM_SEED)]=elapsed;indices[f'{f}_random_{RANDOM_SEED}']=o[:10000]
 np.savez_compressed(out/'retained_indices.npz',**indices);save(out/'original_source_quality.json',basequality)
 # Existing FULL results remain identities/timing-qualified, not new matches.
 p3=json.loads((root/'p3/pairs_01/execution_receipt.json').read_text());historical_full=[x for x in p3['records'] if x['mode']=='OFF']
 save(out/'historical_full.json',dict(P2={str(f):old[f]['result'] for f in FRAMES},P3_OFF=historical_full,warning='P2 trace+FitnessON vs P3 traceOFF+FitnessOFF: do not treat timings as identical'))
 env=os.environ.copy()
 for k in list(env):
  if k.startswith('BD_') or k.startswith('P3_'):del env[k]
 env.update(P3_FITNESS='OFF',LD_LIBRARY_PATH=str(lib.parent)+':/usr/local/cuda/lib64',TMPDIR=str(out),PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1')
 log=(out/'matcher.log').open('x');proc=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,env=env);records=[];count=0
 def call(f,n,method,seed,stage):
  nonlocal count
  assert count<72
  tick=time.perf_counter_ns();points=sources[f];req=dict(old[f]['request']);materialize=0.;write=0.;sampletime=0.
  if method=='full':source=Path(req['source_path']);selected=points;indexsha=None
  else:
   key=(f,method,seed);ix=orders[key][:n];assert len(np.unique(ix))==n;t=time.perf_counter_ns();selected=np.ascontiguousarray(points[ix],dtype='<f4');materialize=(time.perf_counter_ns()-t)*1e-6;source=out/'current_source.bin';t=time.perf_counter_ns()
   with source.open('wb') as fp:fp.write(struct.pack('<Q',n));selected.tofile(fp)
   write=(time.perf_counter_ns()-t)*1e-6;sampletime=order_times[key];indexsha=hashlib.sha256(ix.astype('<i8').tobytes()).hexdigest()
  t=time.perf_counter_ns();q=quality(selected);quality_ms=(time.perf_counter_ns()-t)*1e-6;req['source_path']=str(source);sourcehash=sha(source);requestprep=(time.perf_counter_ns()-tick)*1e-6
  tick=time.perf_counter_ns();proc.stdin.write(json.dumps(req)+'\n');proc.stdin.flush();count+=1;line=proc.stdout.readline();service=(time.perf_counter_ns()-tick)*1e-6
  if not line:raise RuntimeError('MATCHER_TERMINATED')
  z=json.loads(line);assert z['source_n']==n and z['fitness_available']==False and z['fitness'] is None and z['p2_pcl_fit_ms'] is None and not z['legacy_R_generated'],'MATCHER_CONTRACT'
  if method=='full':
   for key in ['final','parameters_internal','native_internal','optimizer_score','score_gradient','score_hessian']:assert np.allclose(z[key],old[f]['result'][key],atol=1e-9,rtol=1e-9),('FULL_PARITY',f,key)
   for key in ['iterations','exit','converged','candidate_valid']:assert z[key]==old[f]['result'][key]
  device=subprocess.run(['nvidia-smi','--query-compute-apps=pid,used_memory','--format=csv,noheader,nounits'],capture_output=True,text=True);memory=None
  for l in device.stdout.splitlines():
   cols=l.split(',')
   if len(cols)>1 and cols[0].strip()==str(proc.pid):
    try:memory=int(cols[1].strip())
    except ValueError:pass
  rec=dict(stage=stage,frame=f,n=n,method=method,seed=seed if method!='full' else None,actual_seed=seed+f if method!='full' else None,request=req,source_sha256=sourcehash,index_sha256=indexsha,quality=q,sampling_order_ms=sampletime,materialize_ms=materialize,source_cache_write_ms=write,coverage_diagnostic_ms=quality_ms,request_prep_excluding_order_ms=requestprep,service_ms=service,result=z,gpu_process_resident_MiB=memory,imu_propagation=False,state_P_commit=False)
  tick=time.perf_counter_ns();(out/f'record_{count:02d}.json').write_text(json.dumps(rec,indent=2)+'\n');rec['record_write_ms']=(time.perf_counter_ns()-tick)*1e-6;records.append(rec);save(out/'progress.json',dict(calls=count,max_calls=72,completed=[{k:x[k] for k in ['stage','frame','n','method','seed']} for x in records]))
  if count==1:
   maps=Path(f'/proc/{proc.pid}/maps').read_text();loaded=[l for l in maps.splitlines() if 'libndt_gpu_matcher.so' in l];assert loaded and all(str(lib) in l for l in loaded);(out/'loaded_library_maps.txt').write_text('\n'.join(loaded)+'\n')
  return rec
 try:
  if a.stage_b_supplement:
   primary=Path(a.stage_b_supplement);base=json.loads((primary/'execution_receipt.json').read_text());assert base['calls']==32
   trigger=[]
   for f in FRAMES:
    rs=[x for x in base['records'] if x['stage']=='A' and x['frame']==f];poses=[np.array(x['result']['final'])[:2,3] for x in rs]+[np.array(old[f]['result']['final'])[:2,3]];pos=np.array(poses);spread=float(np.max(np.linalg.norm(pos[:,None]-pos[None,:],axis=2)));flags=[x['result']['converged'] for x in rs]+[old[f]['result']['converged']]
    if spread>.25 or len(set(flags))>1:trigger.append(dict(frame=f,xy_spread_including_FULL_m=spread,converged_including_FULL=flags))
   assert trigger,'FULL_INCLUSIVE_TRIGGER_NOT_MET'
   save(out/'stageB_decision.json',dict(triggered=True,reasons=trigger,correction='initial primary decision excluded FULL baseline; preserved, corrected to include task-required FULL reference without any OXTS read',primary_receipt_sha256=sha(primary/'execution_receipt.json'),threshold=.25,no_reference_read=True,primary_calls=32,supplement_calls=40,total_planned=72))
   for seed in SPACE_SEEDS[1:]:
    for f in FRAMES:
     for n in COUNTS:call(f,n,'space',seed,'B')
  else:
   for f in FRAMES:
    for n in COUNTS:call(f,n,'space',SPACE_SEEDS[0],'A')
   trigger=[]
   for f in FRAMES:
    rs=[x for x in records if x['stage']=='A' and x['frame']==f];pos=np.array([np.array(x['result']['final'])[:2,3] for x in rs]);spread=float(np.max(np.linalg.norm(pos[:,None]-pos[None,:],axis=2)));flags=[x['result']['converged'] for x in rs]
    if spread>.25 or len(set(flags))>1:trigger.append(dict(frame=f,xy_spread_m=spread,converged=flags))
   save(out/'stageB_decision.json',dict(triggered=bool(trigger),reasons=trigger,threshold=.25,no_reference_read=True))
   for f in [64,66]:call(f,len(sources[f]),'full',0,'A_FULL_SUPPLEMENT')
   if trigger:
    for seed in SPACE_SEEDS[1:]:
     for f in FRAMES:
      for n in COUNTS:call(f,n,'space',seed,'B')
   for f in FRAMES:
    for n in [5000,10000]:call(f,n,'random',RANDOM_SEED,'C')
 finally:
  proc.stdin.close();proc.wait(timeout=30);log.close();save(out/'execution_receipt.json',dict(calls=count,records=records,maximum=72,no_reference_read=True,no_replay=True,no_R=True))
 after={str(p):sha(p) for p in inputs};assert before==after and count in [32,40,72]
 save(out/'identity_after.json',dict(files=after,unchanged=True,calls=count,maximum=72,completed=True));size=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());assert size<50_000_000;print(json.dumps(dict(calls=count,stageB=bool(trigger),result_bytes=size)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--bd',required=True);p.add_argument('--out',required=True);p.add_argument('--stage-b-supplement');main(p.parse_args())
