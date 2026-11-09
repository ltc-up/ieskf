"""Independent full-state update and balance diagnostics, never an estimator input."""
import numpy as np
from scipy.spatial.transform import Rotation

def cross_matrix(v):
 x,y,z=v;return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
def right_jacobian(v):
 t=np.linalg.norm(v);A=cross_matrix(v)
 if t<1e-7:return np.eye(3)-A/2+A@A/6
 return np.eye(3)-(1-np.cos(t))/t**2*A+(t-np.sin(t))/t**3*A@A

def audit_pose_update(x,P,T,R,lever):
 """Frozen XYZ + prior-anchored SO3 chart; no Q/R fitting or gain deletion."""
 H=np.zeros((6,15));H[:3,:3]=np.eye(3);H[:3,6:9]=x.R@cross_matrix(lever);H[3:,6:9]=np.eye(3)
 innovation=np.r_[T[:3,3]-(x.p-x.R@lever),Rotation.from_matrix(x.R.T@T[:3,:3]).as_rotvec()]
 U=H@P@H.T;S=U+R;K=np.linalg.solve(S,H@P).T;d=K@innovation;A=np.eye(15)-K@H;pre_reset=A@P@A.T+K@R@K.T;B=np.eye(15);B[6:9,6:9]=right_jacobian(d[6:9]);post=B@pre_reset@B.T;post=(post+post.T)/2
 L=np.linalg.cholesky(R);white=np.linalg.solve(L,np.linalg.solve(L,U).T).T;white=(white+white.T)/2;values=np.linalg.eigvalsh(white)
 return dict(H=H,innovation=innovation,S=S,K=K,delta=d,P_post=post,P_pre_reset=pre_reset,reset=B,observed_prior=U,whitened_eigenvalues=values,modal_gains=values/(1+values))

def base_position(a,lever):return a[:3]-a[6:15].reshape(3,3)@lever

def difference_balance(prior,posterior,reference,axes):
 """Exact common-axis differences, not truth errors. First row has no predecessor."""
 e0=prior-reference;e1=posterior-reference;prev=np.vstack([e1[0],e1[:-1]]);prevpost=np.vstack([posterior[0],posterior[:-1]]);prevref=np.vstack([reference[0],reference[:-1]]);pre_axes=np.vstack([axes[0],axes[:-1]])
 dot=lambda a,b:np.einsum('ni,ni->n',a,b)
 inherited=dot(prev,axes);new=dot(prior-prevpost-(reference-prevref),axes);update=dot(posterior-prior,axes);basis=dot(prev,axes-pre_axes);prior_error=dot(e0,axes);post_error=dot(e1,axes);closure=prior_error-inherited-new;post_change=np.r_[0.,np.diff(post_error)];cumulative_closure=post_change-new-update-basis
 closure[0]=np.nan;cumulative_closure[0]=np.nan
 return dict(inherited=inherited,new_difference=new,update=update,basis_change=basis,prior_difference=prior_error,posterior_difference=post_error,closure=closure,cumulative_closure=cumulative_closure)

def skip_information_step(replay,checkpoint,event):
 """Fixed-observation intervention: retain integration split, omit one full update.

    A clock step retains held-IMU age and mean/covariance propagation. Removing
    an event outright would also change integration intervals, a second intervention.
    """
 if event['kind'] not in ('wheel','ndt'):raise ValueError('NOT_AN_INFORMATION_EVENT')
 return replay.step(checkpoint,event|dict(kind='clock'))

def pose_variance_components(P,Rpose,lever,axis):
 """Signed contributions; cross covariance is retained and can be negative."""
 A=Rpose@cross_matrix(lever);n=np.asarray(axis);position=float(n@P[:3,:3]@n);attitude=float(n@A@P[6:9,6:9]@A.T@n);cross=float(n@(P[:3,6:9]@A.T+A@P[6:9,:3])@n)
 return np.array([position,attitude,cross])
