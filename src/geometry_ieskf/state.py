from dataclasses import dataclass
from .common import *
@dataclass
class State:
 p:np.ndarray
 v:np.ndarray
 R:np.ndarray
 bg:np.ndarray
 ba:np.ndarray
 t:int=0
 def copy(self):return State(*(x.copy() for x in [self.p,self.v,self.R,self.bg,self.ba]),self.t)
def inject(x,d):return State(x.p+d[:3],x.v+d[3:6],x.R@exp(d[6:9]),x.bg+d[9:12],x.ba+d[12:15],x.t)
def difference(x,ref):return np.r_[x.p-ref.p,x.v-ref.v,log(ref.R.T@x.R),x.bg-ref.bg,x.ba-ref.ba]
def chart_map(d):
 B=np.eye(15);B[6:9,6:9]=jr(d[6:9]);return B
