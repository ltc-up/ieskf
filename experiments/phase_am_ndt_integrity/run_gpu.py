"""Run predeclared jobs on the existing private CUDA service; no matcher vendored."""
import argparse,json,subprocess,time
from pathlib import Path
import numpy as np
from geometry_ieskf.common import dump
from ndt_global.integrity import MapSupport,cluster,classify

def main(a):
    out,pub=Path(a.private_output),Path(a.public_output);jobs=json.loads((out/'bounded_jobs.json').read_text())
    assert len(jobs)==35
    stderr=(out/'gpu_device_access.log').open('w');proc=subprocess.Popen([a.ndt_service],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,text=True)
    results=[];rpc=[];groups=[];batch=time.perf_counter();group_tick=batch
    try:
        for i,j in enumerate(jobs):
            if i%7==0:group_tick=time.perf_counter()
            tick=time.perf_counter();proc.stdin.write(json.dumps(j)+'\n');proc.stdin.flush();line=proc.stdout.readline()
            if not line:raise RuntimeError('CUDA_SERVICE_EARLY_EXIT')
            results.append(json.loads(line));rpc.append((time.perf_counter()-tick)*1000)
            if i%7==6:groups.append((time.perf_counter()-group_tick)*1000)
    finally:
        proc.stdin.close();proc.wait(timeout=20);stderr.close()
    dump(out/'gpu_completed.json',results)
    target=Path(jobs[0]['target_path']);n=int(np.fromfile(target,'<u8',count=1)[0]);support=MapSupport(np.fromfile(target,'<f4',offset=8).reshape(n,3))
    diagnostics=[]
    for offset in range(0,35,7):
        candidates=[]
        for job,m in zip(jobs[offset:offset+7],results[offset:offset+7]):
            source=Path(job['source_path']);q=np.fromfile(source,'<f4',offset=8).reshape(-1,3)
            candidates.append(dict(pose=m['final'],converged=m['converged'],candidate_valid=m['candidate_valid'],fitness=m['fitness'],seed_id=job['case_id'],support=support.diagnose(q,m['final'])))
        modes=cluster(candidates)
        distances=[float(np.linalg.norm(np.array(candidates[x]['pose'])[:3,3]-np.array(candidates[y]['pose'])[:3,3])) for x in range(7) for y in range(x)]
        valid=[c for c in candidates if c['converged'] and c['candidate_valid']]
        conv_dist=[float(np.linalg.norm(np.array(x['pose'])[:3,3]-np.array(y['pose'])[:3,3])) for i,x in enumerate(valid) for y in valid[:i]]
        diagnostics.append(dict(max_converged_separation_m=max(conv_dist,default=0.),modes=modes,candidates=candidates,max_separation_m=max(distances),fitness_range=[min(c['fitness'] for c in candidates),max(c['fitness'] for c in candidates)]))
    dump(out/'multistart_diagnostics.json',diagnostics)
    old=json.loads((pub/'results.json').read_text());old['gpu_sandbox_failure']=old['gpu']
    old['gpu']=dict(attempted=True,planned_jobs=35,returned_results=len(results),success=proc.returncode==0,returncode=proc.returncode,
      rpc_ms={'p50':float(np.median(rpc)),'p95':float(np.percentile(rpc,95)),'max':max(rpc)},
      seven_seed_group_wall_ms=groups,alignment_wall_ms={'p50':float(np.median([m['align_wall_ms'] for m in results])),'max':max(m['align_wall_ms'] for m in results)},
      group_20hz_budget_pass=all(t<=50 for t in groups),end_to_end_20hz_qualified=False)
    old['multistart']=dict(groups=5,clusters_per_group=[len(d['modes']['clusters']) for d in diagnostics],
      ambiguous_groups=sum(d['modes']['classification']=='AMBIGUOUS' for d in diagnostics),
      max_separation_m=max(d['max_separation_m'] for d in diagnostics),
      max_converged_separation_m=max(d['max_converged_separation_m'] for d in diagnostics),
      converged=sum(m['converged'] for m in results),low_fitness_converged=sum(m['converged'] and m['fitness']<1 for m in results),
      low_fitness_competing_groups=sum(sum(any(candidates['converged'] and candidates['fitness']<1 for candidates in [d['candidates'][i] for i in group]) for group in d['modes']['clusters'])>1 for d in diagnostics),
      independent_observations_per_source=0,correct_peak_rate='UNVERIFIED')
    old['real_multimode_rate']='5 predetermined real-source groups only; see multistart'
    dump(pub/'results.json',old);print(json.dumps({'gpu':old['gpu'],'multistart':old['multistart']},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--private-output',required=True);p.add_argument('--public-output',required=True);p.add_argument('--ndt-service',required=True);main(p.parse_args())
