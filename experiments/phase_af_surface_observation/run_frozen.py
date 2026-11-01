"""Frozen surface diagnostics; input contracts stay in local ignored storage.

No AD cost, Hessian, or posterior enters this implementation. Its stored pose
increments are candidate coordinates only. No continuous state recursion.
"""
import argparse,sys,time,json,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'phase_ac_geometry_capture'))
from compare_states import load_first
from geometry_ieskf.common import C,dump,sha
from geometry_ieskf.state import inject
from geometry_ieskf.partition import POSE
from geometry_ieskf.surface_map import SurfaceMap
from geometry_ieskf.surface_observation import *
from synthetic_cases import error_covariance

def score(obs,x,z,P):
 r,H,J,R=obs.linearize(x,z);I=H.T@R.solve(H);A=H[:,POSE];L=np.linalg.cholesky(P[np.ix_(POSE,POSE)]);Iw=L.T@I[np.ix_(POSE,POSE)]@L
 return dict(samples=len(r),residual_abs_quantiles_m=np.percentile(abs(r),[50,90,95,99,100]),residual_rms_m=float(np.sqrt(np.mean(r*r))),shared_quadratic=R.quadratic(r),pose_singular_values_m_rad=np.linalg.svd(A,compute_uv=False),prior_white_information_eigenvalues=np.linalg.eigvalsh(Iw),raw_pose_information_eigenvalues=np.linalg.eigvalsh(I[np.ix_(POSE,POSE)]),raw_information=I,residual=r,H=H,Jplane=J,noise=R)

def run(D,O):
 gate=json.load(open(O/'synthetic_results.json'));assert gate['synthetic_gate']=='PASS'
 aa,data,contract,event,frame,source,prior,P,factor,query=load_first(D);assert event['header_ns']==event['predicted']['state_time_ns']
 endpoints=json.load(open(D/'phase_ad_association_objective/precision_results.json'));assert endpoints[0]['seed_id']=='nominal' and endpoints[6]['seed_id']=='axis2_plus'
 candidates={'prediction':np.zeros(15),'G1':np.array(endpoints[0]['z']),'G2':np.array(endpoints[6]['z']),'AA_partial':np.array(event['delta'])};ys=[]
 for z in candidates.values():
  T=factor.transform(inject(prior,z));ys.append(source@T[:3,:3].T+T[:3,3])
 summary=dict(uncertainty_scope='TRANSFERRED_SYNTHETIC_SCENARIO_NOT_MAP_CALIBRATION',scales=[],candidate_scores=[],transactions=[],data_leakage=True,continuous_updates=0);private=dict(prior=event['predicted'],candidate_increments=candidates,input_time_ns=event['header_ns'],source_hash=frame['source_sha256'],target_hash=contract['target_sha256']);rawtarget=query.target
 for side in [C['source_block_m'],C['source_block_m']/2,C['source_block_m']*2]:
  t=time.perf_counter();sm=SurfaceMap(rawtarget,np.concatenate(ys),side,error_covariance(True));summary['scales'].append(dict(**sm.metrics,extraction_s=time.perf_counter()-t));dump(O/('surface_map_'+str(side)+'.json'),dict(metrics=sm.metrics,groups=sm.groups,patches=sm.patches,unresolved=sm.unresolved,invalid=sm.invalid,plane_parameters={k:vars(v) for k,v in sm.planes.items()}))
  if side==C['source_block_m']:baseline=sm
  z=np.zeros(15);assoc=sm.associate(prior,source,factor);summary['scales'][-1]['prediction_support']=assoc['metrics']
  if len(assoc['source_indices']):
   ix=assoc['source_indices'];obs=SurfaceObservation(source[ix],ix,assoc['plane_ids'],sm.planes,factor);r,H,J,R=obs.linearize(prior);p=linear_surface_update(P,H,r,R);summary['scales'][-1]['conditional_Z_sigma']=float(np.sqrt(p['P_chart'][2,2]));summary['scales'][-1]['information_spectrum']=np.linalg.eigvalsh(p['information'][np.ix_(POSE,POSE)])
 sm=baseline;common=sm.associate(prior,source,factor);private['prediction_association']=common
 if not len(common['source_indices']):summary['real_update_status']='NO_QUALIFIED_SURFACE_SUPPORT';dump(O/'frozen_private.json',private);dump(O/'frozen_summary.json',summary);return
 for name,z in candidates.items():
  x=inject(prior,z);own=sm.associate(x,source,factor);private[name]={'association':own,'scores':{}}
  for label,a in [('prediction_frozen',common),('own_selected',own)]:
   if not len(a['source_indices']):continue
   ix=a['source_indices'];obs=SurfaceObservation(source[ix],ix,a['plane_ids'],sm.planes,factor);q=score(obs,x,z,P);noise=q.pop('noise');private[name]['scores'][label]=q;pub={k:v for k,v in q.items() if k not in ['raw_information','residual','H','Jplane']};pub.update(candidate=name,association=label,support=a['metrics']);summary['candidate_scores'].append(pub)
 ix=common['source_indices'];obs=SurfaceObservation(source[ix],ix,common['plane_ids'],sm.planes,factor);r,H,J,R=obs.linearize(prior);proposal=linear_surface_update(P,H,r,R);transaction=atomic_surface_update(prior,P,proposal,common['metrics']['good']);private['proposal']=proposal;private['transaction']=dict(accepted=transaction['accepted'],reason=transaction['reason'],P=transaction['P'],state=vars(transaction['state']))
 for mode,noise in [('shared_surface',R),('incorrectly_independent_effective_diagonal',SharedPlaneNoise(common['plane_ids'],np.zeros_like(J),sm.planes,R.D+np.einsum('ij,ij->i',J,np.array([sm.planes[str(k)].covariance@j for k,j in zip(common['plane_ids'],J)]))))]:
  p=linear_surface_update(P,H,r,noise);tx=atomic_surface_update(prior,P,p,common['metrics']['good']);d=p['delta'];Knp=np.linalg.solve(P[np.ix_(POSE,POSE)],P[np.ix_(POSE,[3,4,5,9,10,11,12,13,14])]).T
  summary['transactions'].append(dict(mode=mode,accepted_in_isolated_transaction=tx['accepted'],reason=tx['reason'],committed_to_continuous_filter=False,Z_sigma_prior=float(np.sqrt(P[2,2])),Z_sigma_conditional_linear=float(np.sqrt(p['P_chart'][2,2])),vz_sigma_prior=float(np.sqrt(P[5,5])),vz_sigma_conditional_linear=float(np.sqrt(p['P_chart'][5,5])),proposed_delta_z_m=float(d[2]),proposed_delta_vz_m_s=float(d[5]),proposed_delta_velocity_norm_m_s=float(np.linalg.norm(d[3:6])),proposed_delta_bg_norm_rad_s=float(np.linalg.norm(d[9:12])),proposed_delta_ba_norm_m_s2=float(np.linalg.norm(d[12:15])),conditional_recovery_error=float(np.linalg.norm(d[[3,4,5,9,10,11,12,13,14]]-Knp@d[POSE])),Joseph_relative_error=p['Joseph_relative_error'],posterior_min_eigenvalue=float(np.linalg.eigvalsh(p['P_chart'])[0]),actual_committed_delta_vz_m_s=0.))
 # Exact-map alias extraction check after all baseline analysis; coordinates/normals unchanged.
 t=time.perf_counter();copy=SurfaceMap(np.repeat(rawtarget,2,axis=0),np.concatenate(ys),C['source_block_m'],error_covariance(True));ca=copy.associate(prior,source,factor);assert len(copy.groups)==len(sm.groups)
 for a,b in zip(sm.groups,copy.groups):
  for k in ['anchor','normal','target_ids']:np.testing.assert_array_equal(a[k],b[k])
 np.testing.assert_array_equal(ca['source_indices'],common['source_indices']);np.testing.assert_array_equal(ca['target_ids'],common['target_ids']);summary['target_duplicate_test']=dict(copies=2,exact_same_surfaces_and_association=True,elapsed_s=time.perf_counter()-t)
 repeat=np.repeat(np.arange(len(ix)),8);alias=SurfaceObservation(source[ix][repeat],ix[repeat],common['plane_ids'][repeat],sm.planes,factor);rr,HH,JJ,RR=alias.linearize(prior);pp=linear_surface_update(P,HH,rr,RR);summary['source_duplicate_test']=dict(copies=8,information_difference_norm=float(np.linalg.norm(pp['information']-proposal['information'])),posterior_difference_norm=float(np.linalg.norm(pp['P_chart']-proposal['P_chart'])))
 summary['real_update_status']='DISCARDED_ISOLATED_LINEAR_DIAGNOSTIC';dump(O/'frozen_private.json',private);dump(O/'frozen_summary.json',summary)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--data-root',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();run(Path(a.data_root),Path(a.output))
