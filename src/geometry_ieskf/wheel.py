from .state import *
class WheelObservation:
 def __init__(self,lever,config=C):self.lever=np.array(lever);self.c=config
 def model(self,x,omega):
  vb=x.R.T@x.v;w=omega-x.bg;pred=(vb-np.cross(w,self.lever))[0]
  H=np.zeros((1,15));H[0,3:6]=x.R.T[0];H[0,6:9]=skew(vb)[0];H[0,9:12]=-skew(self.lever)[0]
  return pred,H
 def correct(self,x,P,omega,z,variance,age=0.):
  pred,H=self.model(x,omega);res=z-pred;info={'predicted':float(pred),'measurement':float(z),'innovation':float(res)}
  reason=None
  if not np.isfinite([z,variance]).all() or variance<0:reason='INVALID'
  elif abs(age)>self.c['wheel_max_age_s']:reason='STALE'
  elif abs(z)<self.c['wheel_zero_m_s'] and abs(pred)>self.c['wheel_zero_conflict_prediction_m_s']:reason='ZERO_CONFLICT'
  elif abs(res)>self.c['wheel_absolute_residual_limit_m_s']:reason='SLIP_OR_MISMATCH'
  if reason:return x,P,{**info,'accepted':False,'reason':reason}
  var=variance+self.c['wheel_slip_sigma_m_s']**2;S=float((H@P@H.T)[0,0]+var);K=P@H.T/S;d=K[:,0]*res
  B=chart_map(d);A=np.eye(15)-K@H;Pc=A@P@A.T+var*K@K.T;Pc=B@Pc@B.T
  return inject(x,d),(Pc+Pc.T)/2,{**info,'accepted':True,'reason':'ACCEPT','nis_model_conditional':res**2/S,'delta':d.tolist()}
