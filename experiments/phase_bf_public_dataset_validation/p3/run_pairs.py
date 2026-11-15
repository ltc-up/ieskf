"""Bounded Fitness ON/OFF timings: 3 frames x 2 modes x 3 repeats =18.
Qualified original library; no filtering, replay, R creation or pointcloud output.
"""
import argparse,hashlib,importlib.util,json,os,subprocess,time
from pathlib import Path
import numpy as np
FRAMES=(60,65,76)
REPEATS=3
BASE='0ed77cb8f80a5a170eb5dac644ed6df5abbf6712'
LIB_SHA='0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c'
TEMPLATE_SHA='9fd499e21ce43cdbfba3422f5621fe0a4666eb723cb97e07c34bfac5d1a3e72e'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def patch(s,p2patch):
 s=p2patch(s)
 changes=[
 ('#include <chrono>','#include <chrono>\n#include <cuda_runtime_api.h>'),
 ('c.enable_pcl_fit_score=true;','const char* fv=std::getenv("P3_FITNESS");bool fitness_enabled=(!fv || std::string(fv)=="ON");if(fv && std::string(fv)!="ON" && std::string(fv)!="OFF")throw std::runtime_error("invalid P3_FITNESS");c.enable_pcl_fit_score=fitness_enabled;'),
 ('auto t=Clock::now();auto z=matcher->align(source,seed);','auto p3_sync=Clock::now();if(cudaDeviceSynchronize()!=cudaSuccess)throw std::runtime_error("pre sync failure");double p3_pre_sync=ms(p3_sync);auto t=Clock::now();auto z=matcher->align(source,seed);'),
 ('double wall=ms(t);t=Clock::now();','double wall=ms(t);p3_sync=Clock::now();if(cudaDeviceSynchronize()!=cudaSuccess)throw std::runtime_error("post sync failure");double p3_post_sync=ms(p3_sync);t=Clock::now();'),
 ('double qm=ms(t);J result=', 'double qm=ms(t);p3_sync=Clock::now();if(cudaDeviceSynchronize()!=cudaSuccess)throw std::runtime_error("post quality sync failure");double p3_quality_sync=ms(p3_sync);J result='),
 ('std::cout<<result.dump()<<std::endl;', 'bool p3_on=!std::getenv("P3_FITNESS") || std::string(std::getenv("P3_FITNESS"))=="ON";result["fitness_available"]=p3_on;result["fitness_mode"]=p3_on?"ON":"OFF";result["p3_pre_sync_ms"]=p3_pre_sync;result["p3_post_align_sync_ms"]=p3_post_sync;result["p3_post_quality_sync_ms"]=p3_quality_sync;result["legacy_R_generated"]=false;if(!p3_on){result["fitness"]=nullptr;result["fitness_complete"]=nullptr;result["p2_pcl_fit_ms"]=nullptr;}std::cout<<result.dump()<<std::endl;')]
 for x,y in changes:
  if s.count(x)!=1:raise ValueError('P3_PATCH_CONTRACT')
  s=s.replace(x,y)
 return s

def equivalence(result,original):
 e={}
 for k in ['final','parameters_internal','optimizer_score','native_internal','score_gradient','score_hessian','last_step_norm','last_relative_objective_change','kappa_at_float_pose']:
  x=np.asarray(result[k]);y=np.asarray(original[k]);e[k]=dict(max_abs=float(np.max(abs(x-y))),equal=bool(np.allclose(x,y,atol=1e-9,rtol=1e-9)))
 for k in ['iterations','exit','converged','candidate_valid','source_n','valid_pairs','candidate_pairs','inliers_at_float_pose','neighbor_points_at_float_pose','seed_float']:
  e[k]=dict(equal=result[k]==original[k])
 return e

def main(a):
 root=Path(a.root);bd=Path(a.bd);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
 if os.statvfs(out).f_bavail*os.statvfs(out).f_frsize<1<<30:raise ValueError('SPACE_GUARD')
 lib=bd/'build/p0/libndt_gpu_matcher.so';template=bd/'private/match_p0.cpp';assert sha(lib)==LIB_SHA and sha(template)==TEMPLATE_SHA
 spec=importlib.util.spec_from_file_location('p2run',Path(__file__).parents[1]/'p2/run_five.py');p2=importlib.util.module_from_spec(spec);spec.loader.exec_module(p2)
 p2receipt=json.loads((root/'p2/five_01/execution_receipt.json').read_text());p2id=json.loads((root/'p2/five_01/identity_after.json').read_text());assert p2id['calls']==5 and p2id['all_equivalent']
 old={r['frame']:r for r in p2receipt['records'] if r['frame'] in FRAMES};assert set(old)==set(FRAMES)
 inputs=[lib,template,root/'p2/five_01/execution_receipt.json']
 for frame in FRAMES:
  req=old[frame]['request'];assert req['max_iterations']==35 and req['q']==50 and req['bd_config']==dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')
  for name in ['source_path','target_path']:
   f=Path(req[name]);assert sha(f)==p2id['files'][str(f)];inputs.append(f)
 before={str(p):sha(p) for p in set(inputs)};source=out/'wrapper.cpp';source.write_text(patch(template.read_text(),p2.patch_wrapper))
 env=os.environ.copy()
 for k in list(env):
  if k.startswith('BD_') or k.startswith('P3_'):del env[k]
 env.update(TMPDIR=str(out),PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=str(lib.parent)+':/usr/local/cuda/lib64')
 binary=out/'matcher_fitness_toggle';command=['g++','-std=c++17','-O2','-DNDEBUG','-I'+str(bd/'private/matcher_source/include'),'-I/usr/include/eigen3','-I/usr/local/cuda/include',str(source),'-L'+str(lib.parent),'-Wl,-rpath,'+str(lib.parent),'-lndt_gpu_matcher','-L/usr/local/cuda/lib64','-Wl,-rpath,/usr/local/cuda/lib64','-lcudart','-o',str(binary)]
 build=subprocess.run(command,env=env,capture_output=True,text=True);(out/'build.log').write_text(build.stdout+build.stderr);assert build.returncode==0,'BUILD_FAILURE'
 save(out/'identity_before.json',dict(baseline=BASE,files=before,new_wrapper_sha256=sha(source),new_binary_sha256=sha(binary),compile_command=command,trace_enabled=False,frames=FRAMES,repeats=REPEATS,maximum_align_calls=18))
 procs={};logs={};records=[];count=0
 try:
  for mode in ['ON','OFF']:
   logs[mode]=(out/f'{mode}.log').open('x');procs[mode]=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=logs[mode],text=True,env=dict(env,P3_FITNESS=mode))
  for repeat in range(REPEATS):
   for fi,frame in enumerate(FRAMES):
    pair={};order=['ON','OFF'] if (repeat+fi)%2==0 else ['OFF','ON']
    for mode in order:
     proc=procs[mode];req=old[frame]['request'];tick=time.perf_counter_ns();encoded=json.dumps(req);encode_ms=(time.perf_counter_ns()-tick)*1e-6;tick=time.perf_counter_ns();count+=1
     proc.stdin.write(encoded+'\n');proc.stdin.flush();line=proc.stdout.readline();service_ms=(time.perf_counter_ns()-tick)*1e-6
     if not line:raise RuntimeError('MATCHER_TERMINATED')
     tick=time.perf_counter_ns();z=json.loads(line);decode_ms=(time.perf_counter_ns()-tick)*1e-6
     maps=Path(f'/proc/{proc.pid}/maps').read_text();loaded=[l for l in maps.splitlines() if 'libndt_gpu_matcher.so' in l];assert loaded and all(str(lib) in l for l in loaded),'LIBRARY_RESOLUTION';(out/f'{mode}_library_maps.txt').write_text('\n'.join(loaded)+'\n')
     assert z['fitness_available']==(mode=='ON') and z['legacy_R_generated']==False
     if mode=='OFF':assert z['fitness'] is None and z['fitness_complete'] is None and z['p2_pcl_fit_ms'] is None
     else:assert np.isfinite(z['fitness']) and np.isclose(z['fitness'],old[frame]['result']['fitness'],atol=1e-9,rtol=1e-9)
     eq=equivalence(z,old[frame]['result']);rec=dict(repeat=repeat,frame=frame,mode=mode,pair_order=order,result=z,historical_equivalence=eq,request=req,encode_ms=encode_ms,service_ms=service_ms,decode_ms=decode_ms)
     tick=time.perf_counter_ns();(out/f'{repeat}_{frame}_{mode}.json').write_text(json.dumps(rec,indent=2)+'\n');rec['result_write_ms']=(time.perf_counter_ns()-tick)*1e-6;records.append(rec);pair[mode]=z
     save(out/'progress.json',dict(calls=count,maximum=18,completed=[dict(repeat=x['repeat'],frame=x['frame'],mode=x['mode']) for x in records]))
     if not all(v['equal'] for v in eq.values()):raise RuntimeError('OUTPUT_DIFFERENCE_STOP')
    assert all(v['equal'] for v in equivalence(pair['OFF'],pair['ON']).values()),'ON_OFF_DIFFERENCE_STOP'
 finally:
  for proc in procs.values():
   proc.stdin.close();proc.wait(timeout=30)
  for f in logs.values():f.close()
  save(out/'execution_receipt.json',dict(calls=count,maximum_calls=18,completed=len(records),records=records,no_replay=True,no_state_P_commit=True,no_R_construction=True,no_trace=True))
 after={str(p):sha(p) for p in inputs};assert before==after and count==18 and len(records)==18
 save(out/'identity_after.json',dict(unchanged=True,files=after,calls=count,all_equivalent=True))
 size=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());assert size<10_000_000
 print(json.dumps(dict(calls=count,all_equivalent=True,fitness_off_missing_not_zero=True,result_bytes=size)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--bd',required=True);p.add_argument('--out',required=True);main(p.parse_args())
