"""Private-input adapter and aggregate diagnostics; no production source import."""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from compare_states import load_first
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.association_objective import CompactHuberMixture,solve_mixture
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.information_audit import decompose,curvature_summary,polish
from geometry_ieskf.capture import starting_seeds
from geometry_ieskf.state import inject
from geometry_ieskf.partition import margins

def load_case(D):
 aa,data,contract,e,f,source,prior,P,factor,raw=load_first(D)
 reference=NearestValidCorrespondence(raw.target);ad=json.load(open(D/'phase_ad_association_objective/precision_results.json'));roots={'G1':np.array(ad[0]['z']),'G2':np.array(ad[6]['z'])}
 return source,prior,P,factor,reference,roots,ad

def metrics(o,z):
 d=decompose(o,z);curv=curvature_summary(o,z,d['H_white']);x=inject(o.prior,z);angles=Rotation.from_matrix(x.R).as_euler('xyz');pre=Rotation.from_matrix(o.prior.R).as_euler('xyz')
 return {'z':np.array(z),'p_map':x.p,'RPY_map':angles,'delta_XYZ':z[:3],'delta_RPY':(angles-pre+np.pi)%(2*np.pi)-np.pi,'decomposition':d,'curvature':curv,'margins':margins(z),'cost':o.cost(z),'gradient_white_inf':float(np.max(abs(o.L.T@o.gradient(z))))}

def run_fit(o,seed,O,name):
 start=time.perf_counter()
 def stream(row):
  with open(O/(name+'.iterations.jsonl'),'a') as f:f.write(json.dumps(row,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v.item())+'\n')
 raw=solve_mixture(o,seed['z'],on_stage=stream);z,h=polish(o,raw['z']);m=metrics(o,z)
 status='INTERIOR_STATIONARY_PROBE' if m['gradient_white_inf']<=C['convergence_whitened_step'] and min(m['margins'])>1e-5 and m['decomposition']['metrics']['support_good'] and m['curvature']['SPD'] else 'NOT_QUALIFIED'
 r={'seed_id':seed['id'],'raw':raw,'polish':h,'final':m,'status':status,'wall_ms':1000*(time.perf_counter()-start),'filter_updates':0}
 dump(O/(name+'.json'),r);print(name,status,raw['status'],m['cost'],m['gradient_white_inf'],m['decomposition']['metrics']['support_n'],r['wall_ms'],flush=True);return r
