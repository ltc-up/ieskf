"""Opt-in single-packet likelihood contract; legacy defaults are untouched.

This corrects absent queue compensation, not the uncalibrated fitness mapping.
"""
import numpy as np
from scipy.stats import chi2
from .ndt_pose_observation import engineering_R
from .external_pose import ExternalPose
from .posterior import valid_prior

def packet_covariance(fitness,components):
 raw=engineering_R(fitness,smoothing_steps=1)
 return np.diag([raw[0,0] if k in ('x','y','z') else raw[2,2] for k in components])

def assess_integrity(center,hypotheses,geometry,config):
 from ndt_global.integrity import cluster
 result=dict(eligible=False,matching_converged=bool(center['converged']),geometry_covariance=False,global_peak_correctness='UNKNOWN')
 if not center['candidate_valid'] or not center['converged']:return result|dict(reason='CENTRAL_MATCH_NOT_QUALIFIED')
 if geometry.get('rank',0)<config['minimum_geometry_rank']:return result|dict(reason='PARTIAL_OR_WEAK_GEOMETRY')
 if geometry.get('nearby_fraction',0)<config['minimum_support_fraction']:return result|dict(reason='MAP_SUPPORT_INSUFFICIENT')
 groups=cluster([dict(pose=h['final'],converged=h['converged'],candidate_valid=h['candidate_valid']) for h in hypotheses],config['cluster_translation_m'],config['cluster_rotation_rad'])
 if len(groups['clusters'])>1:
  qualified=sum(bool(h['converged'] and h['candidate_valid']) for h in hypotheses)
  if config.get('ambiguity_policy','reject')=='project_consensus' and qualified>=config['required_converged_hypotheses']:
   return result|dict(eligible=True,reason='AMBIGUOUS_PARTIAL_PROJECTION_REQUIRED',clusters=groups['clusters'],qualified_hypotheses=qualified)
  return result|dict(reason='AMBIGUOUS',clusters=groups['clusters'])
 qualified=sum(bool(h['converged'] and h['candidate_valid']) for h in hypotheses)
 if qualified<config['required_converged_hypotheses']:return result|dict(reason='INSUFFICIENT_HYPOTHESES',qualified_hypotheses=qualified)
 return result|dict(eligible=True,reason='LOCAL_CONSENSUS_NOT_GLOBAL_CERTIFICATE',clusters=groups['clusters'],qualified_hypotheses=qualified)

class InnovationGuardedPose(ExternalPose):
 def __init__(self,lever,components,probability=.9999):
  super().__init__(lever,components)
  if not 0<probability<1:raise ValueError('INVALID_NOMINAL_PROBABILITY')
  self.probability=probability
 def correct(self,x,P,T,R,projection=None):
  try:
   valid_prior(P);r,H=self.residual(x,T);R=np.asarray(R,float)
   original_R=R.copy()
   if projection is not None:
    U=np.asarray(projection,float)
    if U.ndim!=2 or U.shape[1]!=len(r) or np.linalg.matrix_rank(U)!=len(U):raise ValueError('INVALID_PROJECTION')
    r=U@r;H=U@H;R=U@R@U.T
   if R.shape!=(len(r),len(r)) or not np.allclose(R,R.T):raise ValueError('INVALID_R')
   np.linalg.cholesky(R);S=H@P@H.T+R;np.linalg.cholesky(S);nis=float(r@np.linalg.solve(S,r));limit=float(chi2.ppf(self.probability,len(r)))
   if not np.isfinite(nis):raise ValueError('NONFINITE_NIS')
  except (ValueError,np.linalg.LinAlgError) as err:return x.copy(),P.copy(),dict(accepted=False,reason='AV_INVALID_OBSERVATION:'+str(err),covariance_updates=0,covariance_calibrated=False)
  if nis>limit:return x.copy(),P.copy(),dict(accepted=False,reason='AV_INNOVATION_INCONSISTENT',innovation=r.tolist(),R=R.tolist(),conditional_NIS=nis,nominal_NIS_limit=limit,covariance_updates=0,covariance_calibrated=False)
  y,Q,info=super().correct(x,P,T,original_R,projection)
  return y,Q,info|dict(likelihood_contract='SINGLE_UNIQUE_PACKET',nominal_NIS_limit=limit,NIS_probability_calibrated=False,observation_rank=len(r))


def hypothesis_consensus_projection(prior_rotation,hypotheses,rotation_scale_m=10.,relative_tolerance=1e-6):
 """Nullspace of candidate differences in the actual prior-anchored pose chart.

 This is a hypothesis-invariant measurement subspace, NOT calibrated covariance
 or a proof that the candidate set contains the correct physical pose.
 """
 from scipy.spatial.transform import Rotation
 values=[]
 for h in hypotheses:
  T=np.asarray(h['final'],float);values.append(np.r_[T[:3,3],Rotation.from_matrix(prior_rotation.T@T[:3,:3]).as_rotvec()])
 D=np.asarray(values)[1:]-values[0];B=np.diag([1.]*3+[rotation_scale_m]*3);normalized=D@B.T
 _,singular,Vt=np.linalg.svd(normalized,full_matrices=True);rank=int(sum(singular>max((singular[0] if len(singular) else 0)*relative_tolerance,1e-12)))
 if rank==0:return None,dict(ambiguity_rank=0,observed_rank=6)
 U=Vt[rank:]@B
 return U,dict(ambiguity_rank=rank,observed_rank=len(U),normalization_m=rotation_scale_m,projected_candidate_difference_max=float(np.max(abs(D@U.T))),chart='map translation and prior-body local rotation',statistical_covariance=False)
