"""Known infinite plane; sampled records do not add tangential surface information."""
import argparse
from pathlib import Path
import numpy as np
from case_tools import *
from geometry_ieskf.state import State
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.density_audit import FrozenNormalsView
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();O=Path(a.output);O.mkdir(exist_ok=True)
T=np.array([[x,y,0.] for x in np.arange(-10.,10.01,.5) for y in np.arange(-10.,10.01,.5)]);source=np.array([[x,y,-1.] for x in np.arange(-7.5,7.51,1.) for y in np.arange(-7.5,7.51,1.)]);prior=State(np.array([0.,0.,1.]),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));factor=GeometryFactor([0,0,0]);std=np.repeat(C['initial_std_p_v_theta_bg_ba'],3);P=np.diag(std**2)
ref=NearestValidCorrespondence(T);ref.cached[:]=True;ref.valid[:]=True;ref.normals[:]=[0,0,1.]
arms={'A':np.arange(len(T)),'B_duplicate2':np.repeat(np.arange(len(T)),2),'C_half':np.arange(len(T))[np.arange(len(T))%2==0],'D_halfspace_duplicate':np.r_[np.arange(len(T)),np.flatnonzero(T[:,0]>=0)]}
results=[]
for name,ids in arms.items():
 q=FrozenNormalsView(ref,ids);o=CompactHuberMixture(prior,P,source,factor,q);truth=metrics(o,np.zeros(15));dump(O/(name+'_truth.json'),truth);r=run_fit(o,{'id':'nominal','z':np.zeros(15)},O,name);results.append({'arm':name,'target_records':len(ids),'truth':truth,'fit':r})
dump(O/'synthetic_results.json',results)
