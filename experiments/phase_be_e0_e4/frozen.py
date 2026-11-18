"""E1/E2 same fit Source/Target/seed; common held-out geometric validation."""
import argparse,json,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from runtime import Service,Validator,read_cloud,write_cloud,partition
from geometry_ieskf.directional_shadow import ndt_information
p=argparse.ArgumentParser();p.add_argument('--paths',type=Path,required=True);p.add_argument('--requests',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--limit',type=int);a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
c=json.load(open(a.paths));req=json.load(open(a.requests));req=req[:a.limit] if a.limit else req
ndt=Service(c['ndt'],a.out/'ndt.log');gicp=Service(c['gicp'],a.out/'gicp.log');val=Validator(req[0]['target_path']);pool=ThreadPoolExecutor(2)
with (a.out/'results.jsonl').open('w') as out:
 for i,r in enumerate(req):
    tick=time.perf_counter_ns();src=read_cloud(r['source_path']);fit,hold=partition(src);path=a.out/'fit.bin';write_cloud(path,fit)
    request=dict(r,source_path=str(path),max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
    f=pool.submit(ndt.call,request);g=pool.submit(gicp.call,request);N,nt=f.result();G,gt=g.result()
    nv=val.cost(hold,N['final']);gv=val.cost(hold,G['final']);row=dict(index=i,header=r['header'],fit_n=len(fit),hold_n=len(hold),ndt=N,gicp=G,ndt_ms=nt,gicp_ms=gt,ndt_validation=nv,gicp_validation=gv,diagnostic=ndt_information(N['score_hessian'],np.array(N['parameters_internal']).reshape(6)[3:]),cycle_ms=(time.perf_counter_ns()-tick)*1e-6)
    out.write(json.dumps(row)+'\n');out.flush()
    if i%100==0:print(i,len(req),flush=True)
ndt.close();gicp.close();pool.shutdown()
