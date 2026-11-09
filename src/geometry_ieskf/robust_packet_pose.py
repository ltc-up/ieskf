"""Opt-in full-state Gaussian scale-mixture pose likelihood.

Conditional noise R/lambda, lambda~Beta(2,1) on(0,1). No component is more
informative than the single-packet Gaussian; latent uncertainty is marginalized.
This is a declared robust model, not empirically calibrated NDT covariance.
"""
from functools import lru_cache
import numpy as np
from scipy.special import logsumexp
from .external_pose import ExternalPose
from .posterior import valid_prior
from .state import inject,chart_map
from .common import C

@lru_cache(maxsize=8)
def quadrature(order):
 if order<8:raise ValueError('QUADRATURE_ORDER')
 x,w=np.polynomial.legendre.leggauss(order);x=(x+1)/2;w=w/2;precision=x**4
 # p(lambda)=2lambda and d(lambda)/dx=4x^3. Fourth-power mapping resolves
 # small precision without truncating the heavy tail at an empirical floor.
 log_prior_weight=np.log(w)+np.log(8.)+7*np.log(x)
 return precision,log_prior_weight

def scale_mixture_moments(P,H,R,residual,order=128):
 P=valid_prior(P);H=np.asarray(H,float);R=np.asarray(R,float);r=np.asarray(residual,float);m=len(r)
 if R.shape!=(m,m) or H.shape!=(m,15) or not np.allclose(R,R.T) or not np.isfinite(np.r_[r,H.ravel(),R.ravel()]).all():raise ValueError('INVALID_MIXTURE_INPUT')
 np.linalg.cholesky(R);lam,log_prior=quadrature(order);noise=R[None,:,:]/lam[:,None,None];S=(H@P@H.T)[None,:,:]+noise;chol=np.linalg.cholesky(S);rhs=np.c_[H@P,r];solution=np.linalg.solve(S,np.broadcast_to(rhs,(len(lam),m,16)));K=solution[:,:,:15].transpose(0,2,1);nis=np.einsum('i,ni->n',r,solution[:,:,-1]);logdet=2*np.log(np.diagonal(chol,axis1=1,axis2=2)).sum(axis=1);logw=log_prior-.5*(nis+logdet+m*np.log(2*np.pi));weights=np.exp(logw-logsumexp(logw));means=np.einsum('nij,j->ni',K,r);A=np.eye(15)[None,:,:]-K@H;conditional=A@P@A.transpose(0,2,1)+K@noise@K.transpose(0,2,1);mean=weights@means;centered=means-mean;within=np.einsum('n,nij->ij',weights,conditional);between=np.einsum('n,ni,nj->ij',weights,centered,centered);cov=within+between;cov=(cov+cov.T)/2;effective_K=np.einsum('n,nij->ij',weights,K);gradient=-solution[:,:,-1];centered_gradient=gradient-weights@gradient;sensitivity=effective_K+np.einsum('n,ni,nj->ij',weights,centered,centered_gradient)
 return dict(mean=mean,covariance=cov,K=effective_K,sensitivity_gain=sensitivity,within=within,between=between,weights=weights,precision=lam,expected_precision=float(weights@lam),conditional_NIS=float(r@np.linalg.solve(H@P@H.T+R,r)))

class ScaleMixturePose(ExternalPose):
 def __init__(self,lever,components,order=128):super().__init__(lever,components);self.order=order
 def correct(self,x,P,T,R,projection=None):
  info=dict(accepted=False,covariance_calibrated=False,components=self.components,covariance_updates=0,likelihood_contract='SINGLE_PACKET_BETA_PRECISION_MIXTURE')
  try:
   r,H=self.residual(x,T);R=np.asarray(R,float)
   if projection is not None:
    U=np.asarray(projection,float)
    if U.ndim!=2 or U.shape[1]!=len(r) or np.linalg.matrix_rank(U)!=len(U):raise ValueError('INVALID_PROJECTION')
    r=U@r;H=U@H;R=U@R@U.T
   result=scale_mixture_moments(P,H,R,r,self.order);d=result['mean'];info.update(innovation=r.tolist(),R=R.tolist(),K=result['K'].tolist(),local_residual_jacobian=result['sensitivity_gain'].tolist(),delta_proposed=d.tolist(),conditional_NIS=result['conditional_NIS'],expected_precision=result['expected_precision'],precision_nodes=result['precision'].tolist(),precision_weights=result['weights'].tolist(),observation_rank=len(r),latent_between_mean_covariance_trace=float(np.trace(result['between'])))
   if not np.isfinite(d).all():raise ValueError('NONFINITE')
   if np.linalg.norm(d[6:9])>C['max_total_rotation_rad']:raise ValueError('ATTITUDE_LINEARIZATION_DOMAIN')
   if np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s'] or np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']:raise ValueError('BIAS_CHANGE_DOMAIN')
   B=chart_map(d);Q=B@result['covariance']@B.T;Q=(Q+Q.T)/2;valid_prior(Q);y=inject(x,d)
  except (ValueError,np.linalg.LinAlgError) as err:return x.copy(),P.copy(),info|dict(reason='AV_MIXTURE:'+str(err))
  before=dict(p=x.p.tolist(),v=x.v.tolist(),R=x.R.tolist(),bg=x.bg.tolist(),ba=x.ba.tolist());after=dict(p=y.p.tolist(),v=y.v.tolist(),R=y.R.tolist(),bg=y.bg.tolist(),ba=y.ba.tolist())
  return y,Q,info|dict(accepted=True,reason='EXTERNAL_POSE_SCALE_MIXTURE_UPDATE',delta=d.tolist(),covariance_updates=1,P_after_native=Q.tolist(),native_state_before=before,native_state_after=after,covariance_formula='TOTAL_MIXTURE_COVARIANCE_THEN_RIGHT_RESET')
