"""Exactly five fixed-input registrations, using the historical shared library.
No filter import/update/replay. Wrapper edits export timers only, never NDT math.
All full private inputs/results remain outside the public repository.
"""
import argparse,hashlib,json,os,struct,subprocess,time
from pathlib import Path
import numpy as np
FRAMES=(60,64,65,66,76)
BASE='6e43ecb0a53470f229e7d0882ccb4e7950ee14ec'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def patch_wrapper(s):
    replacements=[
      ('auto j=J::parse(line);','auto p2_start=Clock::now();double p2_target_read=0,p2_region=0,p2_source=0;auto j=J::parse(line);'),
      ('auto target=load(j["target_path"]);','auto p2_tr=Clock::now();auto target=load(j["target_path"]);p2_target_read=ms(p2_tr);'),
      ('matcher->setActiveRegion(ar);','auto p2_rg=Clock::now();matcher->setActiveRegion(ar);p2_region=ms(p2_rg);'),
      ('auto source=load(j["source_path"]);','auto p2_sr=Clock::now();auto source=load(j["source_path"]);p2_source=ms(p2_sr);double p2_prepare=ms(p2_start);'),
      ('{"quality_extra_ms",qm}', '{"quality_extra_ms",qm},{"p2_pcl_fit_ms",z.pcl_fit_ms},{"p2_target_read_ms",p2_target_read},{"p2_region_ms",p2_region},{"p2_source_read_ms",p2_source},{"p2_prepare_ms",p2_prepare}')]
    for a,b in replacements:
        if s.count(a)!=1:raise ValueError('WRAPPER_PATCH_CONTRACT')
        s=s.replace(a,b)
    return s

def main(a):
    root=Path(a.root);bd=Path(a.bd);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    if os.statvfs(out).f_bavail*os.statvfs(out).f_frsize<1<<30:raise ValueError('SPACE_GUARD')
    historical_binary=bd/'runtime/match_p0';lib=bd/'build/p0/libndt_gpu_matcher.so';template=bd/'private/match_p0.cpp'
    expected={historical_binary:'8ae76d7ece996cf89f498fba2519ba4c73d76b8f64beb69ffd5884a243ea1c08',lib:'0c229305a733c467a5a63019489768f4ea0b626f449cff23c56b707ccb0ed54c',template:'9fd499e21ce43cdbfba3422f5621fe0a4666eb723cb97e07c34bfac5d1a3e72e'}
    for p,h in expected.items():assert sha(p)==h,('HISTORICAL_IDENTITY',p)
    data=root/'kitti/derived_time_aligned';rows={r['frame']:r for r in map(json.loads,(root/'results/kitti/B0_run1/frames.jsonl').open()) if r['frame'] in FRAMES};assert set(rows)==set(FRAMES)
    online=np.load(data/'online.npz');T=online['T_IMU_velo'];map_path=data/'map.bin';assert sha(map_path)=='293300a932d4ad7cb9cb6d7225d48dcd6a78b68f78c3a45b504f4146f28e3f2d'
    historical_config=json.loads((root/'results/kitti/B0_run1/config.json').read_text());assert historical_config['matcher_sha256']==expected[historical_binary]
    assert historical_config['ndt']==dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')
    inputs=list(expected)+[map_path,data/'online.npz',data/'config.json',root/'results/kitti/B0_run1/frames.jsonl'];prepared=[]
    for i in FRAMES:
        tick=time.perf_counter_ns();source=data/'sources'/f'{i:03d}.bin';raw=root/'kitti/raw/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data'/f'{i:010d}.bin'
        rawpoints=np.fromfile(raw,dtype='<f4').reshape(-1,4)[:,:3];recreated=np.asarray(rawpoints@T[:3,:3].T+T[:3,3],'<f4');payload=source.read_bytes();n=struct.unpack('<Q',payload[:8])[0]
        assert n==rows[i]['ndt']['source_n'] and payload[8:]==recreated.tobytes(),('SAVED_SOURCE_NOT_EQUIVALENT',i)
        inputs += [source,raw];prepared.append(dict(frame=i,source=str(source),source_sha256=sha(source),raw_sha256=sha(raw),count=n,identity_check_read_transform_compare_ms=(time.perf_counter_ns()-tick)*1e-6))
    before={str(p):sha(p) for p in inputs}
    source=out/'wrapper.cpp';source.write_text(patch_wrapper(template.read_text()))
    env=os.environ.copy();env.update(TMPDIR=str(out),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='4')
    for k in list(env):
        if k.startswith('BD_'):del env[k]
    env['BD_TRACE']=str(out/'iterations.jsonl');env['LD_LIBRARY_PATH']=str(lib.parent)
    binary=out/'matcher_timers';command=['g++','-std=c++17','-O2','-DNDEBUG','-I'+str(bd/'private/matcher_source/include'),'-I/usr/include/eigen3',str(source),'-L'+str(lib.parent),'-Wl,-rpath,'+str(lib.parent),'-lndt_gpu_matcher','-o',str(binary)]
    build=subprocess.run(command,env=env,capture_output=True,text=True);(out/'build.log').write_text(build.stdout+build.stderr);assert build.returncode==0,'BUILD_FAILED'
    save(out/'identity_before.json',dict(baseline=BASE,files=before,wrapper_sha256=sha(source),diagnostic_binary_sha256=sha(binary),compile_command=command,prepared=prepared,environment={k:env[k] for k in ['BD_TRACE','LD_LIBRARY_PATH','TMPDIR','OMP_NUM_THREADS']},configuration=historical_config['ndt'],extra_align_calls=0,filter_updates=0))
    records=[];count=0;proc=None
    with (out/'matcher.log').open('x') as log:
        proc=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,env=env)
        try:
            for item in prepared:
                i=item['frame'];r=rows[i];seed=np.asarray(r['seed']);req=dict(case_id=f'KITTI_{i}',header=r['scan_epoch_ns'],group=f'kitti0002:{i}',scene='KITTI0002',source_path=item['source'],target_path=str(map_path),target_sha=sha(map_path),roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,float(seed[2,3]-10),float(seed[2,3]+30),120,i+1],seed_common=seed.tolist(),q=50.,max_iterations=35,bd_config=historical_config['ndt'])
                tick=time.perf_counter_ns();encoded=json.dumps(req);encode_ms=(time.perf_counter_ns()-tick)*1e-6;tick=time.perf_counter_ns();count+=1;proc.stdin.write(encoded+'\n');proc.stdin.flush();line=proc.stdout.readline();service_ms=(time.perf_counter_ns()-tick)*1e-6
                if not line:raise RuntimeError('MATCHER_TERMINATED')
                tick=time.perf_counter_ns();result=json.loads(line);decode_ms=(time.perf_counter_ns()-tick)*1e-6
                maps=Path(f'/proc/{proc.pid}/maps').read_text();loaded=[l for l in maps.splitlines() if 'libndt_gpu_matcher.so' in l];assert loaded and all(str(lib) in l for l in loaded),'LIBRARY_RESOLUTION';(out/'loaded_library_maps.txt').write_text('\n'.join(loaded)+'\n')
                eq={};tolerance=dict(final=(2e-6,2e-6),parameters_internal=(1e-8,1e-8),fitness=(1e-8,1e-8),optimizer_score=(1e-7,1e-8),native_internal=(1e-9,1e-7),score_gradient=(1e-5,1e-7),score_hessian=(1e-3,1e-7),last_step_norm=(1e-8,1e-7),last_relative_objective_change=(1e-8,1e-7))
                for k,(at,rt) in tolerance.items():
                    x=np.asarray(result[k]);y=np.asarray(r['ndt'][k]);eq[k]=dict(max_abs=float(np.max(abs(x-y))),equal=bool(np.allclose(x,y,atol=at,rtol=rt)),atol=at,rtol=rt)
                for k in ['iterations','exit','converged','candidate_valid','source_n','valid_pairs','candidate_pairs','fitness_complete','seed_float']:
                    eq[k]=dict(equal=result[k]==r['ndt'][k])
                rec=dict(frame=i,request=req,result=result,equivalence=eq,encode_ms=encode_ms,service_ms=service_ms,decode_ms=decode_ms,imu_advance='NOT_EXECUTED',filter_updates=0)
                tick=time.perf_counter_ns();(out/f'frame_{i}.json').write_text(json.dumps(rec,indent=2)+'\n');rec['result_write_ms']=(time.perf_counter_ns()-tick)*1e-6;records.append(rec)
                save(out/'progress.json',dict(calls=count,allowed=list(FRAMES),completed=[x['frame'] for x in records],stopped_on_equivalence_failure=not all(v['equal'] for v in eq.values())))
                if not all(v['equal'] for v in eq.values()):raise RuntimeError('HISTORICAL_EQUIVALENCE_FAILED_NO_MORE_CALLS')
        finally:
            if proc:
                proc.stdin.close();proc.wait(timeout=30)
            save(out/'execution_receipt.json',dict(calls=count,completed=[x['frame'] for x in records],no_warmup=True,no_replay=True,no_state_P_commit=True,records=records))
    after={str(p):sha(p) for p in inputs};assert before==after,'INPUT_MODIFIED'
    assert count==5 and len(records)==5
    save(out/'identity_after.json',dict(files=after,unchanged=True,all_equivalent=True,calls=count))
    total=sum(p.stat().st_size for p in out.rglob('*') if p.is_file());assert total<10_000_000,('RESULT_BUDGET',total)
    print(json.dumps(dict(calls=count,all_equivalent=True,input_unchanged=True,result_bytes=total)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--bd',required=True);p.add_argument('--out',required=True);main(p.parse_args())
