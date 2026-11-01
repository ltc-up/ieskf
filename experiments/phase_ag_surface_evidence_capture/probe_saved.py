"""Postdeclared endpoint witness, no reoptimization or independent truth claim."""
import argparse,sys,csv,json,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'phase_ac_geometry_capture'))
from compare_states import load_first
from run_surface_capture import load_surfaces,obs_for,retention
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.partition import POSE
from geometry_ieskf.state import inject
from geometry_ieskf.surface_capture import FrozenSurfaceObjective

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--collection-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();D=Path(a.data_root);O=Path(a.output);root=Path(a.collection_root)
 aa,data,contract,event,frame,source,prior,P,factor,query=load_first(D);s=load_surfaces(D/'phase_af_surface_observation',5.,np.unique(query.target,axis=0));saved=json.load(open(O/'capture_5.0.json'));z=np.array(saved['z']);base=s.associate(inject(prior,z),source,factor);o=FrozenSurfaceObjective(prior,P,obs_for(base,source,s,factor));Lp=np.linalg.cholesky(P[np.ix_(POSE,POSE)]);probes=[]
 for j in range(6):
  for sign in [-1,1]:
   d=np.zeros(15);d[POSE]=sign*C['convergence_whitened_step']*Lp[:,j];b=s.associate(inject(prior,z+d),source,factor);probes.append(dict(axis=j,sign=sign,retention=retention(base,b),support=b['metrics']))
 deriv=[];g=o.L.T@o.gradient(z)
 for h in [1e-5,1e-6]:
  fd=np.array([(o.cost(z+h*o.L[:,j])-o.cost(z-h*o.L[:,j]))/(2*h) for j in range(15)]);deriv.append(dict(step_white=h,analytical=g,finite_difference=fd,max_abs_difference=float(np.max(abs(g-fd)))))
 refs=[]
 for name in ['vectornav_gps.csv','analysis_stage1/reference_trajectory.csv']:
  p=root/'r2_lap_dataset'/name
  with p.open() as f:rows=list(csv.DictReader(f))
  times=[float(r['stamp']) for r in rows];refs.append(dict(path=str(p),sha256=sha(p),rows=len(rows),time_range=[min(times),max(times)],begins_after_first_frame_s=min(times)-event['header_ns']/1e9))
 dump(O/'endpoint_probe_private.json',dict(predeclared_after_endpoint_before_probe=True,probes=probes,directional_derivatives=deriv,later_recording_references=refs));print('Probes changed memberships',sum(not p['retention']['identical'] for p in probes),'of12; derivative differences',[(v['step_white'],v['max_abs_difference']) for v in deriv])
if __name__=='__main__':main()
