"""Diagnostic fixed XY/yaw score optimization via unchanged local CUDA SDK.

This is NOT the production NDT optimizer. SDK sources/adapter stay private.
The service must return the original score and additive XYZ/RxRyRz derivatives.
"""
import argparse,json,subprocess,time,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
from geometry_ieskf.common import C,dump

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--score-service',required=True);ap.add_argument('--only-groups',default='AC');a=ap.parse_args();O=Path(a.output);jobs=json.load(open(O/'ndt_jobs.json'));full=[json.loads(l) for l in open(O/'ndt_results.jsonl')];allout=[]
 for index,j in enumerate(jobs):
  if j['group'] not in a.only_groups:continue
  tick=time.perf_counter();proc=subprocess.Popen([a.score_service,str(O/'ndt_jobs.json'),str(index)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True);trace=[]
  def score(p):
   proc.stdin.write(json.dumps(np.asarray(p).tolist())+'\n');proc.stdin.flush();line=proc.stdout.readline()
   if not line:raise RuntimeError('score service failure '+proc.stderr.read())
   v=json.loads(line);trace.append(dict(parameters=p,**v));return v
  seed=np.array(j['seed_common']);angles=Rotation.from_matrix(seed[:3,:3]).as_euler('xyz');r0=Rotation.from_euler('xyz',angles);scale=max(1,jobs[index]['source_path'] and int(np.fromfile(j['source_path'],'<u8',count=1)[0]))
  def param(u):return np.r_[seed[0,3],seed[1,3],seed[2,3]+u[0],Rotation.from_euler('xyz',angles+np.r_[u[1:3],0.]).as_euler('XYZ')]
  def jacparam(u):
   B=np.zeros((6,3));B[2,0]=1
   for k in [1,2]:
    d=np.eye(3)[k]*1e-6;plus=param(u+d);minus=param(u-d);B[3:,k]=((plus[3:]-minus[3:]+np.pi)%(2*np.pi)-np.pi)/2e-6
   return B
  cache={}
  def data(u):
   key=tuple(u)
   if key not in cache:
    v=score(param(u));cache[key]=(-v['score']/scale,-jacparam(u).T@np.array(v['gradient'])/scale)
   return cache[key]
  atfull=np.array(full[index]['parameters_internal']).ravel();parity=score(atfull)['score'];assert abs(parity-full[index]['optimizer_score'])<=1e-6*max(1,abs(parity))
  g0=data(np.zeros(3))[1];fd=np.array([(data(np.eye(3)[k]*1e-5)[0]-data(-np.eye(3)[k]*1e-5)[0])/2e-5 for k in range(3)]);grad_error=float(np.max(abs(g0-fd)))
  def guard(u):return 1-np.linalg.norm((r0.inv()*Rotation.from_euler('xyz',angles+np.r_[u[1:3],0.])).as_rotvec())**2/C['max_total_rotation_rad']**2
  # Separate diagnostic optimizer; no assertion of GLL/production-path equivalence.
  def diff(u):return np.array([(data(u+np.eye(3)[k]*1e-5)[0]-data(u-np.eye(3)[k]*1e-5)[0])/2e-5 for k in range(3)])
  sol=minimize(lambda u:data(u)[0],np.zeros(3),jac=diff,method='SLSQP',bounds=[(-C['max_total_translation_m'],C['max_total_translation_m']),(-C['max_total_rotation_rad'],C['max_total_rotation_rad']),(-C['max_total_rotation_rad'],C['max_total_rotation_rad'])],constraints=[dict(type='ineq',fun=guard)],options=dict(maxiter=35,ftol=1e-10));u=sol.x;v=score(param(u));feasible=abs(u[0])<=C['max_total_translation_m']+1e-8 and guard(u)>=-1e-8;g=diff(u);interior=abs(u[0])<C['max_total_translation_m']-1e-5 and guard(u)>1e-5;qualified=bool(sol.success and feasible and interior and np.max(abs(g))<=1e-4);out=dict(case_id=j['case_id'],u=u,final_parameters=param(u),initial_cost=data(np.zeros(3))[0],final_cost=data(u)[0],solver_success=bool(sol.success),solver_message=str(sol.message),iterations=int(sol.nit),feasible=bool(feasible),interior=bool(interior),qualified_local_interior=qualified,gradient_final=g,gradient_FD_error_at_seed=grad_error,archive_score_parity_error=parity-full[index]['optimizer_score'],valid_pairs=v['valid_pairs'],score_trace=trace,elapsed_s=time.perf_counter()-tick,scope='fixed XY/yaw diagnostic minimizing NEGATIVE unchanged CUDA score, central FD1e-5; API angular derivative mismatch NOT used; no production optimizer equivalence');allout.append(out);proc.stdin.close();proc.wait();print(j['case_id'],qualified,'dZ',u[0],sol.message,flush=True)
 dump(O/'constrained_ndt_private.json',allout)
if __name__=='__main__':main()
