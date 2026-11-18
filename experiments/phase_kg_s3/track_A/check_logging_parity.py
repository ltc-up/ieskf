"""Prove the approved passive A1 logging change preserves numerical outputs."""
import importlib.util,sys,subprocess,tempfile,json,hashlib
from pathlib import Path
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[3]/'src'),str(Path(__file__).parent)]
import score_map as new
from geometry_ieskf.state import State
from scipy.spatial.transform import Rotation

def check():
 text=subprocess.check_output(['git','show','02e3ee9:experiments/phase_kg_s3/track_A/score_map.py'],text=True)
 with tempfile.TemporaryDirectory() as temp:
  path=Path(temp)/'old.py';path.write_text(text)
  spec=importlib.util.spec_from_file_location('kg_s3_pre_logging_map',path);old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
  records=[];rng=np.random.default_rng(19)
  for rank in [4,6]:
   for index in range(6):
    H=-np.diag([100,80,60,30,20,10] if rank==6 else [100,80,0,30,20,0]);G=rng.normal(size=(18,6))*(1 if index<3 else 100)
    def geometry(q):return dict(hessian=H,block_gradients=G)
    state=State(rng.normal(size=3)*.1,np.array([3.,0,0]),Rotation.from_euler('XYZ',[.12,.08,-.2]).as_matrix(),np.zeros(3),np.zeros(3),index)
    P=np.eye(15)*.04;P[0,3]=P[3,0]=.01;T=np.eye(4);T[:3,:3]=Rotation.from_euler('XYZ',[.14,.07,-.18]).as_matrix();T[:3,3]=state.p+[.1,-.03,.02]
    a,A,ia=old.DirectionalObserver(np.zeros(3),geometry,'s','t').correct(state,P,T,None)
    b,B,ib=new.DirectionalObserver(np.zeros(3),geometry,'s','t').correct(state,P,T,None)
    identical=all(np.array_equal(getattr(a,k),getattr(b,k)) for k in ['p','v','R','bg','ba']) and np.array_equal(A,B) and ia['accepted']==ib['accepted']
    if not identical:raise AssertionError('PASSIVE_LOGGING_CHANGED_RESULT')
    records.append(dict(rank=rank,index=index,bitwise_same_mean_fullP=True,new_fields_present=all(k in ib for k in ['R_q','floor_variance_eigenvalues','full_chart_R','projection'])))
  return dict(old_freeze='02e3ee9',cases=records,all_bitwise_same=True,scope='synthetic same-input passive logging only')
if __name__=='__main__':print(json.dumps(check(),indent=2))
