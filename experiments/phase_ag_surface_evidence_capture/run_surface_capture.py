"""Read AF frozen factors; conditional optimization only, no map refit/recursion."""
import argparse,sys,json,time,hashlib,numpy as np
from pathlib import Path
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'phase_ac_geometry_capture'))
from compare_states import load_first
from geometry_ieskf.common import C,dump
from geometry_ieskf.state import inject,chart_map
from geometry_ieskf.surface_observation import Plane,SurfaceObservation,linear_surface_update
from geometry_ieskf.surface_map import SurfaceMap
from geometry_ieskf.surface_capture import FrozenSurfaceObjective,constrained_surface_solve
from geometry_ieskf.partition import margins

def load_surfaces(folder,side,target):
 a=json.load(open(folder/('surface_map_'+str(side)+'.json')));s=SurfaceMap.__new__(SurfaceMap);s.config=C;s.side=side;s.target=target;s.planes={k:Plane(k,*[np.array(v[n]) for n in ['normal','anchor','tangent','covariance']]) for k,v in a['plane_parameters'].items()};s.groups=a['groups'];s.support_ids=np.unique(np.concatenate([g['target_ids'] for g in s.groups]));s.target_group=np.full(len(target),-1,int)
 for i,g in enumerate(s.groups):s.target_group[g['target_ids']]=i
 s.tree=cKDTree(target[s.support_ids]);s.metrics=a['metrics'];return s

def signature(a):return hashlib.sha256(json.dumps(list(zip(a['source_indices'].tolist(),a['plane_ids'].tolist()))).encode()).hexdigest()
def retention(a,b):
 A=set(zip(map(int,a['source_indices']),a['plane_ids']));B=set(zip(map(int,b['source_indices']),b['plane_ids']));return dict(source_plane_jaccard=len(A&B)/max(1,len(A|B)),identical=A==B,entered=len(B-A),exited=len(A-B))
def obs_for(a,source,s,factor):
 ix=a['source_indices'];return SurfaceObservation(source[ix],ix,a['plane_ids'],s.planes,factor)
def run(D,O):
 aa,data,contract,event,frame,source,prior,P,factor,query=load_first(D);assert event['header_ns']==event['predicted']['state_time_ns'];target=np.unique(query.target,axis=0);L=np.linalg.cholesky(P);results=[]
 for side in [5.,2.5,10.]:
  start=time.perf_counter();s=load_surfaces(D/'phase_af_surface_observation',side,target);z=np.zeros(15);a=s.associate(prior,source,factor);history=[];seen={};status='OUTER_BUDGET_EXHAUSTED';stable=False
  for outer in range(20):
   if not a['metrics']['good']:status='LOW_SURFACE_SUPPORT';break
   o=FrozenSurfaceObjective(prior,P,obs_for(a,source,s,factor));sol=constrained_surface_solve(o,z,60);nextz=sol['z'];b=s.associate(inject(prior,nextz),source,factor);turn=retention(a,b);next_o=FrozenSurfaceObjective(prior,P,obs_for(b,source,s,factor)) if len(b['source_indices']) else None;new_cert=next_o.local(nextz)[3] if next_o else None
   rec=dict(outer=outer,association_before=a,association_after=b,local_solve=sol,association_change=turn,own_reassociated_certificate=new_cert,frozen_R_reference='ORIGINAL_PREDICTION',cost_comparison='different source-plane/R hypotheses across outer steps',relative_state_white=float(np.linalg.norm(np.linalg.solve(L,nextz-z))));history.append(rec);z=nextz
   if not sol['qualified']:status='LOCAL_'+sol['status'];break
   if turn['identical'] and b['metrics']['good'] and new_cert['qualified']:stable=True;status='CONDITIONAL_LOCAL_AND_ASSOCIATION_STABLE';a=b;break
   sig=signature(b)
   if sig in seen:
    prev,prevz=seen[sig]
    if np.linalg.norm(np.linalg.solve(L,z-prevz))<=C['convergence_whitened_step']:status='ASSOCIATION_CYCLE';rec['cycle_period']=outer-prev;break
   seen[sig]=(outer,z.copy());a=b
  final_a=s.associate(inject(prior,z),source,factor);final_o=FrozenSurfaceObjective(prior,P,obs_for(final_a,source,s,factor)) if len(final_a['source_indices']) else None;final_certificate=final_o.local(z)[3] if final_o else None
  posterior=None
  if final_o:
   r,H=final_o.data(z);reference=linear_surface_update(P,H,r-H@z,final_o.noise);G=chart_map(z);posterior=G@reference['P_chart']@G.T
  out=dict(side_m=side,status=status,association_stable=stable,z=z,certificate=final_certificate,history=history,final_association=final_a,conditional_GN_covariance_not_submitted=posterior,elapsed_s=time.perf_counter()-start,real_uncertainty='UNCALIBRATED_AF_SYNTHETIC_TRANSFER',state_or_P_commits=0)
  dump(O/('capture_'+str(side)+'.json'),out);results.append(out);print(side,status,'outer',len(history),'dZ',z[2],'inner_total',sum(len(h['local_solve']['history']) for h in history),flush=True)
 dump(O/'binding_private.json',dict(source_hash=frame['source_sha256'],target_hash=contract['target_sha256'],native_time_ns=event['header_ns'],prior=event['predicted'],configuration=C,baseline_AF_single_linear= json.load(open(D/'phase_af_surface_observation/frozen_private.json'))['proposal']))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();run(Path(a.data_root),Path(a.output))
