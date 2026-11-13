"""Replay pre-frozen requests using an authorized private instrumented matcher.

Requests may contain private paths/poses; outputs stay in the supplied private root.
No raw data is included with this public research driver.
"""
import argparse,json,os,subprocess
from pathlib import Path
p=argparse.ArgumentParser()
for k in ['matcher','requests','output']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--fixed-min',type=float);a=p.parse_args()
requests=json.loads(a.requests.read_text());a.output.mkdir(exist_ok=False)
grid=[dict(transformation_epsilon=e,relative_objective_epsilon=o,line_search=ls,mode='mt') for o in [0.,.0001,.001] for e in [.001,.01] for ls in [False,True]]
grid.append(dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
(a.output/'grid.json').write_text(json.dumps(grid,indent=2))
for i,c in enumerate(grid):
 d=a.output/f'controlled_{i:02d}';d.mkdir();env=os.environ.copy()
 env.pop('BD_CONFIG',None);env.pop('BD_MAX',None);env.pop('BD_MIN',None)
 env['BD_TRACE']=str(d/'iterations.jsonl')
 if a.fixed_min is not None:env['BD_MIN']=str(a.fixed_min)
 with (d/'stderr.log').open('w') as error,(d/'results.jsonl').open('w') as result:
  proc=subprocess.Popen([str(a.matcher)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=error,text=True,env=env)
  try:
   for r in requests:
    proc.stdin.write(json.dumps(dict(r,bd_config=c))+'\n');proc.stdin.flush()
    line=proc.stdout.readline()
    if not line:raise RuntimeError('matcher stopped; preserve partial output and stderr')
    json.loads(line);result.write(line);result.flush()
   proc.stdin.close();code=proc.wait()
   if code:raise RuntimeError('matcher failed: '+str(code))
  finally:
   if proc.poll() is None:proc.terminate();proc.wait()
