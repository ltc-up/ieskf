from pathlib import Path
import json,sys,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
import os
from .geometry_factor import skew,residual_jacobian,update as prior_update
CONFIG_PATH=Path(os.environ.get('IESKF_CONFIG', str(Path(__file__).with_name('research_defaults.json'))))
C=json.loads(CONFIG_PATH.read_text())
def dump(p,x):
 def clean(a):
  if isinstance(a,np.ndarray):return a.tolist()
  if isinstance(a,np.generic):return a.item()
  raise TypeError(type(a).__name__)
 Path(p).write_text(json.dumps(x,indent=2,default=clean,allow_nan=False))
def sha(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  while b:=f.read(8*1024*1024):h.update(b)
 return h.hexdigest()
def exp(w):return Rotation.from_rotvec(w).as_matrix()
def log(R):return Rotation.from_matrix(R).as_rotvec()
def jr(w):
 a=np.linalg.norm(w);W=skew(w)
 if a<1e-5:return np.eye(3)-(.5-a*a/24)*W+(1/6-a*a/120)*W@W
 return np.eye(3)-(1-np.cos(a))/a**2*W+(a-np.sin(a))/a**3*W@W
