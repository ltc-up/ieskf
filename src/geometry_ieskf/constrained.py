"""Same four norm-ball constraints; independent full and nested block references."""
from .partition import *
def refine_kkt(u,grad,con,jac,hess_cons,max_steps=6):
 u=u.copy();c=con(u);active=np.flatnonzero(c<=1e-5);J=jac(u);mu=nnls(J[active].T,grad(u))[0] if len(active) else np.empty(0);history=[]
 def residual(v,m):return np.r_[grad(v)-jac(v)[active].T@m,con(v)[active]]
 for it in range(max_steps):
  F=residual(u,mu);history.append({'iteration':it,'u':u.copy(),'multipliers':mu.copy(),'residual_inf':float(np.max(abs(F)))})
  if np.max(abs(F))<1e-8:break
  n=len(u);H=np.empty((n,n))
  for j in range(n):
   eps=1e-5*max(1.,abs(u[j]));d=np.eye(n)[j]*eps;H[:,j]=(grad(u+d)-grad(u-d))/(2*eps)
  H=(H+H.T)/2
  for a,m in zip(active,mu):H-=m*hess_cons(a)
  J=jac(u)[active];M=np.block([[H,-J.T],[J,np.zeros((len(active),len(active)))]])
  try:step=np.linalg.solve(M,-F)
  except np.linalg.LinAlgError:break
  used=None
  for alpha in [1.,.5,.25,.125,.0625]:
   uu=u+alpha*step[:n];mm=mu+alpha*step[n:]
   if np.min(mm,initial=0)<-1e-8 or np.min(con(uu),initial=0)<-1e-7:continue
   if np.linalg.norm(residual(uu,mm))<np.linalg.norm(F):u=uu;mu=mm;used=alpha;break
  history[-1]['step_scale']=used
  if used is None:break
 return u,history

def full_constrained(o,z_initial=None):
 L=o.L;history=[]
 def fun(u):
  z=L@u;value=o.cost(z);history.append({'evaluation':len(history),'cost':value,'z':z.copy(),'gradient':o.gradient(z),'constraints':margins(z)});return value
 def grad(u):return L.T@o.gradient(L@u)
 def con(u):return margins(L@u)
 def jac(u):return con_jac_z(L@u)@L
 def hc(i):ix=BOUNDS[i][1];l=BOUNDS[i][2];return -2*L[ix].T@L[ix]/l**2
 begin=time.perf_counter();sol=minimize(fun,np.zeros(15) if z_initial is None else o.Q@z_initial,jac=grad,constraints=[{'type':'ineq','fun':con,'jac':jac}],method='SLSQP',options={'maxiter':200,'ftol':1e-12});raw=L@sol.x;u,polish=refine_kkt(sol.x,grad,con,jac,hc);z=L@u;polished=z.copy()
 if margins(z).min()<0:z=z*(1-1e-10)
 k=kkt(o,z)
 if not sol.success and not k['optimal']:k['status']='SOLVER_FAILURE' if not k['feasible'] else 'FEASIBLE_NOT_CONVERGED'
 return {'z':z,'cost':o.cost(z),'initial_cost':o.cost(np.zeros(15)),'decrease':o.cost(np.zeros(15))-o.cost(z),'kkt':k,'raw_SLSQP_z':raw,'raw_SLSQP_margins':margins(raw),'polished_raw_z':polished,'strict_feasibility_cleanup_used':bool(np.any(z!=polished)),'SLSQP_success':bool(sol.success),'SLSQP_message':str(sol.message),'SLSQP_iterations':int(sol.nit),'history':history,'KKT_polish_history':polish,'wall_ms':1000*(time.perf_counter()-begin)}

class ConditionalNine:
 """Convex inner9 Gaussian problem solved by two nonnegative ball multipliers."""
 def __init__(self,p):
  self.p=p;self.blocks=[(np.arange(3,6),C['max_bias_gyro_change_rad_s']),(np.arange(6,9),C['max_bias_accel_change_m_s2'])];self.calls=0;self.saved=[]
 def solve(self,pose):
  self.calls+=1;m=self.p.K@pose;Ln=self.p.Ln;As=[Ln[ix]/lim for ix,lim in self.blocks];aa=[m[ix]/lim for ix,lim in self.blocks]
  def calc(lam):
   M=np.eye(9);b=np.zeros(9)
   for t,A,a in zip(lam,As,aa):M+=2*t*A.T@A;b-=2*t*A.T@a
   u=np.linalg.solve(M,b);s=[a+A@u for A,a in zip(As,aa)];c=np.array([1-v@v for v in s]);Bs=np.array([A.T@v for A,v in zip(As,s)]);val=-float(.5*u@u-lam@c);H=4*Bs@np.linalg.solve(M,Bs.T)
   return val,c,H,u
  lam=np.zeros(2);start=calc(lam)
  if start[1].min()<0:
   sol=minimize(lambda t:calc(t)[0],lam,jac=lambda t:calc(t)[1],method='L-BFGS-B',bounds=[(0,None)]*2,options={'maxiter':200,'ftol':1e-15,'gtol':1e-12,'maxls':30});lam=sol.x
   for it in range(6):
    val,c,H,u=calc(lam);act=np.flatnonzero((lam>1e-9)|(c< -1e-10))
    if not len(act) or np.max(abs(c[act]))<1e-11:break
    step=np.linalg.solve(H[np.ix_(act,act)],-c[act]);trial=lam.copy();trial[act]+=step
    if np.min(trial)<0:break
    lam=trial
  val,c,H,u=calc(lam);n=m+Ln@u;station=u.copy()
  for l,A,a in zip(lam,As,aa):station+=2*l*A.T@(a+A@u)
  result={'n':n,'u_conditional':u,'cost_extra':float(.5*u@u),'multipliers':lam,'margins':c,'stationarity_inf':float(np.max(abs(station))),'complementarity_inf':float(np.max(abs(lam*c))),'feasible':bool(c.min()>=-1e-7),'calls':self.calls}
  self.saved.append({'pose':pose.copy(),**result});return result

def nested_constrained(o):
 p=Partition(o);inner=ConditionalNine(p);L=p.Lp;cache={};history=[]
 def at(v):
  key=v.tobytes()
  if key not in cache:
   pose=L@v;a=inner.solve(pose);z=p.join(pose,a['n']);cache[key]={'z':z,'cost':o.cost(z),'gradient':L.T@o.gradient(z)[POSE],'inner':a};history.append({'evaluation':len(history),'pose':pose.copy(),**cache[key]})
  return cache[key]
 def grad(v):return at(v)['gradient']
 def con(v):return margins(p.join(L@v,np.zeros(9)))[:2]
 def jac(v):return con_jac_z(p.join(L@v,np.zeros(9)))[:2][:,POSE]@L
 def hc(i):ix=np.arange(3) if i==0 else np.arange(3,6);limit=BOUNDS[i][2];return -2*L[ix].T@L[ix]/limit**2
 begin=time.perf_counter();sol=minimize(lambda v:at(v)['cost'],np.zeros(6),jac=grad,constraints=[{'type':'ineq','fun':con,'jac':jac}],method='SLSQP',options={'maxiter':200,'ftol':1e-12});v,polish=refine_kkt(sol.x,grad,con,jac,hc);result=at(v);z=result['z'].copy();raw=z.copy()
 if margins(z).min()<0:z=z*(1-1e-10)
 k=kkt(o,z)
 return {'z':z,'cost':o.cost(z),'kkt':k,'inner':result['inner'],'raw_z':raw,'strict_feasibility_cleanup_used':bool(np.any(z!=raw)),'SLSQP_success':bool(sol.success),'SLSQP_message':str(sol.message),'SLSQP_iterations':int(sol.nit),'history':history,'KKT_polish_history':polish,'inner_all':inner.saved,'inner_calls':inner.calls,'wall_ms':1000*(time.perf_counter()-begin)}
