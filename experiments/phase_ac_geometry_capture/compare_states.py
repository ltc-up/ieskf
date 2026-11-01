"""AC1: same frozen prior; own-selected and common-fixed objectives distinguished."""
import argparse,json,numpy as np
from pathlib import Path
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.state import inject,chart_map
from geometry_ieskf.objective import state,Objective
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.qualification import TrackedCorrespondence,local_optimality,pair_retention
from geometry_ieskf.partition import margins,POSE

def association(a,b):
 d=pair_retention(a,b);ia={int(k):i for i,k in enumerate(a['source_index'])};ib={int(k):i for i,k in enumerate(b['source_index'])};cs=sorted(ia.keys()&ib.keys());ang=[np.degrees(np.arccos(np.clip(abs(a['n'][ia[k]]@b['n'][ib[k]]),0,1))) for k in cs]
 d['normal_degrees_quantiles']=np.percentile(ang,[50,90,100]).tolist() if ang else [];return d

def evaluate(prior,P,z,corr,factor):
 o=Objective(prior,P,corr,factor);r,J,rho,h=o.data(z);m=local_optimality(P,z,J,r,corr['w']*h);m.update(cost=o.cost(z),prior_cost=float(.5*np.sum(np.linalg.solve(o.L,z)**2)),geometry_cost=float(corr['w']@rho),support=corr['metrics'],engineering_margins=margins(z),delta_pose=z[POSE],delta_other=z[[3,4,5,9,10,11,12,13,14]],prior_white_radius=float(np.linalg.norm(np.linalg.solve(o.L,z))),residual_quantiles=np.percentile(abs(r),[50,90,95,99,100]))
 return m

def load_first(D):
 aa=json.load(open(D/'phase_aa_z_validation/new_replay.json'));data=json.load(open(D/'history/phase_v/results/input_continuous_extension.json'));contract=json.load(open(D/'history/phase_v/results/input_contract.json'));e=aa['lidar_updates'][0];f=next(x for x in data['frames'] if x['header_ns']==e['header_ns']);path=D/'history/phase_v'/f['source_file'];assert sha(path)==f['source_sha256'];assert sha(D/'inputs/map_target.bin')==contract['target_sha256'];source=np.load(path);prior=state(e['predicted']);P=np.array(e['predicted']['P']);factor=GeometryFactor(contract['extrinsic']['lever_body_imu_m']);query=TrackedCorrespondence(np.fromfile(D/'inputs/map_target.bin','<f4',offset=8).reshape(-1,3).astype(float))
 return aa,data,contract,e,f,source,prior,P,factor,query

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();D=Path(a.data_root);O=Path(a.output);O.mkdir(exist_ok=True,parents=True)
 aa,data,contract,e,f,source,prior,P,factor,query=load_first(D);ref=json.load(open(D/'phase_ab_geometry_qualification/representative_diagnostics.json'))[0];zB=np.array(e['delta']);starts=[np.zeros(15),zB,zB+np.array(ref['fixed_extra_z'])];corrs=[query.get(inject(prior,z),source,factor) for z in starts];common=Objective(prior,P,corrs[1],factor);records=[]
 from scipy.spatial.transform import Rotation
 for name,z,corr in zip(['A_prediction','B_AA_committed_partial','C_AB_fixed_final_factor_refined'],starts,corrs):
  m=evaluate(prior,P,z,corr,factor);x=inject(prior,z);yaw=(Rotation.from_matrix(x.R).as_euler('xyz')[2]-Rotation.from_matrix(prior.R).as_euler('xyz')[2]+np.pi)%(2*np.pi)-np.pi
  m.update(name=name,common_AA_final_factor_cost=common.cost(z),delta_z_m=float(z[2]),delta_XY_norm_m=float(np.linalg.norm(z[:2])),delta_map_yaw_rad=float(yaw),association_from_prediction=association(corrs[0],corr));records.append(m)
 dump(O/'three_states.json',records);dump(O/'three_state_binding.json',{'source_sha256':f['source_sha256'],'target_sha256':contract['target_sha256'],'header_ns':f['header_ns'],'prior':e['predicted'],'z_A':starts[0],'z_B':starts[1],'z_C':starts[2]});print([{k:r[k] for k in ['name','cost','common_AA_final_factor_cost','gn_white_norm','newton_decrement','delta_z_m','delta_XY_norm_m','delta_map_yaw_rad','engineering_margins']} for r in records])
if __name__=='__main__':main()
