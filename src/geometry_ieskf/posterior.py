"""Unconstrained fixed-linearization posterior; atomic proposal/commit eligibility."""
from .partition import PERM,margins
from .common import *
from .state import inject,chart_map
from scipy.linalg import cho_factor,cho_solve,solve_triangular

def valid_prior(P):
 P=np.asarray(P)
 if P.shape!=(15,15) or not np.isfinite(P).all():raise ValueError('INVALID_PRIOR_SHAPE_OR_FINITE')
 if np.max(abs(P-P.T))>1e-10*max(1.,np.max(abs(P))):raise ValueError('ASYMMETRIC_PRIOR')
 np.linalg.cholesky((P+P.T)/2)
 return (P+P.T)/2

def block_posterior(P,Hpose,weights):
 P=valid_prior(P);H=np.asarray(Hpose);w=np.asarray(weights)
 if H.ndim!=2 or H.shape[1]!=6 or len(H)==0 or w.shape!=(len(H),) or not np.isfinite(np.r_[H.ravel(),w]).all() or np.any(w<=0):raise ValueError('INVALID_GEOMETRY_LINEARIZATION')
 Pr=PERM@P@PERM.T;Paa=Pr[:6,:6];Pna=Pr[6:,:6];Pnn=Pr[6:,6:];Lp=np.linalg.cholesky(Paa);K=cho_solve(cho_factor(Paa),Pna.T).T;S=Pnn-K@Pna.T;S=(S+S.T)/2;Ln=np.linalg.cholesky(S);A=np.sqrt(w)[:,None]*(H@Lp);M=np.eye(6)+A.T@A;C6=np.linalg.cholesky(M)
 pose_root=Lp@solve_triangular(C6.T,np.eye(6),lower=False);Ppose=pose_root@pose_root.T;root_reordered=np.block([[pose_root,np.zeros((6,9))],[K@pose_root,Ln]]);root_full=PERM.T@root_reordered;Pchart=root_full@root_full.T;Pchart=(Pchart+Pchart.T)/2
 return {'P_chart':Pchart,'root_chart':root_full,'P_pose':Ppose,'K_np':K,'conditional_covariance':S,'Lp':Lp,'M_pose_white':M,'Hpose':H,'weights':w,'covariance_label':'UNCONSTRAINED_LOCAL_LINEAR_GAUSSIAN_OR_CONDITIONAL_IRLS_APPROXIMATION'}

def linear_update(P,Hpose,weights,innovation):
 result=block_posterior(P,Hpose,weights);v=np.asarray(innovation)
 if v.shape!=(len(Hpose),) or not np.isfinite(v).all():raise ValueError('INVALID_INNOVATION')
 rhs=result['Lp'].T@np.asarray(Hpose).T@(np.asarray(weights)*v);pose=result['Lp']@cho_solve(cho_factor(result['M_pose_white']),rhs);delta=PERM.T@np.r_[pose,result['K_np']@pose];return delta,result

def commit(prior,P,delta,Hpose,weights,flags):
 """Never clip state. Reject preserves BOTH state and P. Reset applied exactly once."""
 keep=lambda status,reasons:{'state':prior.copy(),'P':np.asarray(P).copy(),'accepted':False,'status':status,'reasons':reasons,'covariance_updates':0,'NUMERIC_POSTERIOR_VALID':False}
 try:valid_prior(P)
 except (ValueError,np.linalg.LinAlgError):return keep('NUMERIC_FAILURE',['INVALID_PRIOR'])
 z=np.asarray(delta)
 if z.shape!=(15,) or not np.isfinite(z).all():return keep('NUMERIC_FAILURE',['NONFINITE_OR_INVALID_PROPOSAL'])
 feasible=bool(np.min(margins(z))>=0);reason=[]
 if not flags.get('local_converged',False):reason.append('PARTIAL_DECREASE_ONLY')
 if not flags.get('support_good',False):reason.append('LOW_GEOMETRY_SUPPORT')
 if not flags.get('association_stable',False):reason.append('ASSOCIATION_NOT_STABLE')
 if not feasible:reason.append('ENGINEERING_GUARD_EXCEEDED')
 if reason:return keep('REJECTED_KEEP_PRIOR',reason)
 try:
  post=block_posterior(P,Hpose,weights);B=chart_map(z);root=B@post['root_chart'];Pnew=root@root.T;Pnew=(Pnew+Pnew.T)/2;np.linalg.cholesky(Pnew)
  if not np.isfinite(Pnew).all():raise ValueError('NONFINITE_POSTERIOR')
 except (ValueError,np.linalg.LinAlgError):return keep('NUMERIC_FAILURE',['INVALID_POSTERIOR_OR_MEASUREMENT'])
 return {'state':inject(prior,z),'P':Pnew,'P_chart':post['P_chart'],'accepted':True,'status':'QUALIFIED_UNCONSTRAINED_GEOMETRY_UPDATE','reasons':[],'covariance_updates':1,'NUMERIC_POSTERIOR_VALID':True,'STATISTICAL_POSTERIOR_CALIBRATED':False,'posterior_label':post['covariance_label']}
