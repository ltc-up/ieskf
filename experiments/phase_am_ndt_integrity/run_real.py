import argparse,json,time,subprocess,collections,hashlib
from pathlib import Path
import numpy as np
from geometry_ieskf.common import sha,dump
from ndt_global.integrity import MapSupport,GeometryConfig,cluster,classify

def main(a):
    root,out,pub=Path(a.data_root),Path(a.private_output),Path(a.public_output)
    out.mkdir(parents=True,exist_ok=True);pub.mkdir(parents=True,exist_ok=True)
    rows=[json.loads(l) for l in (root/'phase_aj_ndt_pose_update/ndt_transactions.jsonl').read_text().splitlines()]
    data=json.loads((root/'history/phase_v/results/input_continuous_extension.json').read_text())
    target=root/'inputs/map_target.bin';n=int(np.fromfile(target,'<u8',count=1)[0]);points=np.fromfile(target,'<f4',offset=8).reshape(n,3)
    cfg=GeometryConfig();tick=time.perf_counter();support=MapSupport(points,cfg);build_ms=(time.perf_counter()-tick)*1000
    records=[];times=[]
    for i,(row,e) in enumerate(zip(rows,data['frames'])):
        path=root/'history/phase_v'/e['source_file'];assert sha(path)==e['source_sha256'];q=np.load(path)
        assert row['measurement']['header']==e['header_ns']
        tick=time.perf_counter();s=support.diagnose(q,row['measurement']['final']);times.append(1000*(time.perf_counter()-tick))
        c=dict(pose=row['measurement']['final'],converged=row['measurement']['converged'],candidate_valid=row['measurement']['candidate_valid'],fitness=row['measurement']['fitness'],seed_id=hashlib.sha256(json.dumps(row['request']['seed_common']).encode()).hexdigest())
        m=cluster([c]);state=classify(converged=c['converged'],support=s,modes=m)
        records.append(dict(index=i,source_sha256=e['source_sha256'],support=s,modes=m,state=state,fitness=c['fitness'],converged=c['converged']))
        if (i+1)%70==0:print('diagnosed',i+1,flush=True)
    # Fixed first-event missing-map intervention, centered on ORIGINAL seed, not correct answer.
    row=rows[0];q=np.load(root/'history/phase_v'/data['frames'][0]['source_file']);seed=np.array(row['request']['seed_common'])
    kept=points[points[:,0]<seed[0,3]];hole=MapSupport(kept,cfg).diagnose(q,row['measurement']['final'])
    # Predetermined bounded multi-start jobs: center, +/-1.5m x/y, +/-0.1rad yaw.
    from scipy.spatial.transform import Rotation
    jobs=[]
    for index in (0,71,142,213,283):
        row=rows[index]
        for j,(dx,dy,yaw) in enumerate([(0,0,0),(1.5,0,0),(-1.5,0,0),(0,1.5,0),(0,-1.5,0),(0,0,.1),(0,0,-.1)]):
            job=dict(row['request']);T=np.array(job['seed_common']);T[:2,3]+=[dx,dy];T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@T[:3,:3]
            job.update(case_id=f'AM_{index}_{j}',seed_common=T.tolist(),target_path=str(target.resolve()),source_path=str((root/'phase_aj_ndt_pose_update/sources'/f'{index:04d}.bin').resolve()))
            jobs.append(job)
    dump(out/'bounded_jobs.json',jobs)
    service=Path(a.ndt_service);start=time.perf_counter()
    try:
        # One service process; stop on infrastructure failure, preserve all planned jobs.
        res=subprocess.run([str(service)],input=''.join(json.dumps(j)+'\n' for j in jobs),text=True,capture_output=True,timeout=45)
        (out/'gpu_stdout.jsonl').write_text(res.stdout);(out/'gpu_stderr.log').write_text(res.stderr)
        parsed=[]
        for line in res.stdout.splitlines():
            try:parsed.append(json.loads(line))
            except json.JSONDecodeError:pass
        gpu=dict(attempted=True,planned_jobs=len(jobs),returned_results=len(parsed),returncode=res.returncode,success=bool(len(parsed)==len(jobs) and res.returncode==0))
    except (subprocess.TimeoutExpired,OSError) as ex:
        (out/'gpu_stderr.log').write_text(str(ex));gpu=dict(attempted=True,planned_jobs=len(jobs),returned_results=0,success=False,error_type=type(ex).__name__)
    gpu['attempt_wall_s']=time.perf_counter()-start;gpu['alignment_latency_qualified']=False
    ranks=collections.Counter(r['support'].get('rank',0) for r in records);states=collections.Counter(r['state'] for r in records)
    fractions=[r['support'].get('nearby_fraction',0) for r in records]
    summary=dict(qualification='PARTIAL',real_frames=len(records),states=dict(states),rank_histogram=dict(ranks),
      nearby_fraction={'min':min(fractions),'p50':float(np.median(fractions)),'max':max(fractions)},
      low_fitness_converged_unknown=sum(r['fitness']<1 and r['converged'] and r['state']=='UNKNOWN' for r in records),
      real_wrong_peak_rate='UNVERIFIED_NO_INDEPENDENT_TRUTH',real_multimode_rate='UNAVAILABLE_GPU',
      injected_map_hole={'original_fraction':fractions[0],'removed_fraction':hole.get('nearby_fraction',0),'proves_original_hole':False},
      cpu_geometry_ms={'p50':float(np.median(times)),'p95':float(np.percentile(times,95)),'max':max(times),'tree_build_ms':build_ms},
      gpu=gpu,config=cfg.__dict__,input_stream_sha256=sha(root/'history/phase_v/results/input_continuous_extension.json'),
      target_sha256=sha(target),ledger_sha256=sha(root/'phase_aj_ndt_pose_update/ndt_transactions.jsonl'),service_binary_sha256=sha(service),
      per_lidar_validation='UNVERIFIED_NO_POINT_LINEAGE_IN_FROZEN_SOURCE',covariance='NOT_PROVIDED')
    ratios=[r['support']['geometry']['eigenvalues'][0]/r['support']['geometry']['eigenvalues'][-1] for r in records if r['support'].get('geometry') and r['support']['rank']>0]
    summary['normalized_weak_ratio']=dict(min=float(min(ratios)),p50=float(np.median(ratios)),max=float(max(ratios)))
    dump(out/'diagnostics.json',records);dump(out/'map_hole.json',hole);dump(pub/'results.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--private-output',required=True);p.add_argument('--public-output',required=True);p.add_argument('--ndt-service',required=True);main(p.parse_args())
