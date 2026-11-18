"""Budgeted native build: cached FAR-LIO + fixed local dependencies; no network fetch."""
import argparse,json,os,shutil,subprocess,time
from pathlib import Path


def main(a):
    root=Path(a.root).absolute();ref=Path(a.source).absolute();deps=root/'dependencies';runtime=root/'runtime'/a.attempt
    if root.resolve().name != 'phase_bg_far_lio' or '新加卷' not in str(root.resolve()):raise RuntimeError('WRONG_ACTUAL_VOLUME')
    for name in ['tmp','cache','cache/cuda','roslog']: (runtime/name).mkdir(parents=True,exist_ok=True)
    ledger=json.loads((deps/'download_ledger.json').read_text())
    if sum(x['bytes'] for x in ledger)>512*1024**2:raise RuntimeError('DOWNLOAD_BUDGET')
    if shutil.disk_usage(root).free<50*1024**3:raise RuntimeError('LOW_SPACE')
    def src(name):
        paths=[p for p in (deps/name).iterdir() if p.is_dir()]
        if len(paths)!=1:raise ValueError(name)
        return paths[0]
    # Preseed upstream's bootstrap file with its own pinned RAPIDS entry point.
    # No source patch; FetchContent is forced to the audited local trees.
    bootstrap=runtime/'build/tam_odometry/_deps/cucollections-build/CUCO_RAPIDS.cmake'
    bootstrap.parent.mkdir(parents=True,exist_ok=True)
    if not bootstrap.exists():shutil.copyfile(src('rapids')/'RAPIDS.cmake',bootstrap)
    args=['colcon','--log-base',str(runtime/'log_build'),'build','--base-paths',str(ref/'ros_packages'),str(ref/'lib_cpp'),str(deps),
          '--build-base',str(runtime/'build'),'--install-base',str(runtime/'install'),
          '--packages-up-to','tam_odometry','tam_state_estimation_node','--executor','sequential',
          '--event-handlers','console_direct+','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_CUDA=ON',
          '-DCUDA_ARCHITECTURES=120','-DCMAKE_CUDA_ARCHITECTURES=120','-DBUILD_ODOMETRY_EXAMPLES=OFF',
          '-DBUILD_TESTING=OFF','-DVISUALIZATION=OFF','-DFETCHCONTENT_FULLY_DISCONNECTED=ON','-DFETCHCONTENT_UPDATES_DISCONNECTED=ON',
          '-DCPM_LOCAL_PACKAGES_ONLY=ON','-DCPM_DOWNLOAD_LOCATION='+str(src('cpm')/'cmake/CPM.cmake'),
          '-DFETCHCONTENT_SOURCE_DIR_RAPIDS-CMAKE='+str(src('rapids')),
          '-DCCCL_DIR=/usr/local/cuda/lib64/cmake/cccl','-Drosidl_adapter_DIR='+str(deps/'rosidl_adapter_utf8')]
    for cm,name in [('ROBIN_MAP','robin_map'),('GOOGLETEST','googletest'),('SOPHUS','sophus'),('TBB','tbb'),('CUCOLLECTIONS','cuco')]:
        args.append('-DFETCHCONTENT_SOURCE_DIR_'+cm+'='+str(src(name)))
    env=os.environ.copy();env.update(TMPDIR=str(runtime/'tmp'),XDG_CACHE_HOME=str(runtime/'cache'),CUDA_CACHE_PATH=str(runtime/'cache/cuda'),
                                    ROS_LOG_DIR=str(runtime/'roslog'),PYTHONDONTWRITEBYTECODE='1',CMAKE_BUILD_PARALLEL_LEVEL='2',
                                    OPENBLAS_NUM_THREADS='1',CUDACXX='/usr/local/cuda/bin/nvcc',PATH='/usr/local/cuda/bin:'+env['PATH'])
    env['CMAKE_PREFIX_PATH']='/usr/local/cuda:'+env.get('CMAKE_PREFIX_PATH','')
    cert=dict(args=args,environment={k:env[k] for k in ['TMPDIR','XDG_CACHE_HOME','CUDA_CACHE_PATH','ROS_LOG_DIR','CMAKE_BUILD_PARALLEL_LEVEL']},started=time.time(),network_policy='local sources only; upstream bootstrap preseeded; FetchContent disconnected')
    (root/('audit/build_command_'+a.attempt+'.json')).write_text(json.dumps(cert,indent=2)+'\n')
    log=root/('audit/native_build_'+a.attempt+'.log');sizes=[]
    with log.open('ab') as f:
        p=subprocess.Popen(args,env=env,cwd=runtime,stdout=f,stderr=subprocess.STDOUT)
        while p.poll() is None:
            size=sum(x.stat().st_blocks*512 for x in root.rglob('*') if x.is_file())
            sizes.append([time.time(),size])
            if size>20*1024**3 or shutil.disk_usage(root).free<50*1024**3:
                p.terminate();raise RuntimeError('STORAGE_BUDGET_STOP')
            time.sleep(2)
    result=dict(returncode=p.returncode,elapsed_s=time.time()-cert['started'],allocated_samples=sizes)
    (root/('audit/build_result_'+a.attempt+'.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='allocated_samples'}))
    raise SystemExit(p.returncode)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--attempt',required=True);p.add_argument('--root',required=True);p.add_argument('--source',required=True);main(p.parse_args())
