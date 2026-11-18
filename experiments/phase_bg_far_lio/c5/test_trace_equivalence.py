"""No estimator parameter changes: passive observer must be numerically identical."""
import sys,json
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[3];sys.path.insert(0,str(REPO/'src'))
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.state import State
import trace_backend
class Capture:
 def __init__(self):self.rows=[]
 def put(self,r):self.rows.append(r)
def test_synthetic_trace_exact():
 x=State(np.array([1.,2.,3.]),np.array([32.,1.,0.]),np.eye(3),np.zeros(3),np.zeros(3),100);P=np.eye(15)*.1;T=np.eye(4);T[:3,3]=[1.1,1.8,3.];R=np.eye(6)*.3;lever=np.array([1.,.2,.1]);a=ExternalPose(lever,('x','y','z','rx','ry','rz'));b=trace_backend.TraceObserver(lever,'one');log=Capture();trace_backend.LOG=log;trace_backend.WINDOW=(0,1000)
 ya,Qa,ia=a.correct(x,P,T,R);yb,Qb,ib=b.correct(x,P,T,R)
 assert np.array_equal(ya.p,yb.p) and np.array_equal(ya.R,yb.R) and np.array_equal(Qa,Qb) and ia==ib
 assert len(log.rows)==1 and log.rows[0]['uid']=='one';trace_backend.LOG=None
def test_no_trace_outside_window():
 x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),100);b=trace_backend.TraceObserver(np.zeros(3),'two');log=Capture();trace_backend.LOG=log;trace_backend.WINDOW=(101,200);b.correct(x,np.eye(15),np.eye(4),np.eye(6));assert not log.rows;trace_backend.LOG=None
def test_checked_in_core_unchanged():
 import subprocess
 z=subprocess.run(['git','diff','--','src'],cwd=REPO,capture_output=True,text=True,check=True);assert not z.stdout
