"""Executable protocol tests against separately built existing MIT dependency."""
import unittest,os,tempfile,subprocess,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
class SmallGicpProtocolTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  binary=os.environ.get('AW_GICP_BINARY')
  if not binary:raise unittest.SkipTest('set AW_GICP_BINARY to built authorized adapter')
  cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name);grid=np.linspace(-3,3,20);a,b=np.meshgrid(grid,grid);a=a.ravel();b=b.ravel();S=np.r_[np.c_[a,b,np.zeros_like(a)],np.c_[a,np.zeros_like(a),b],np.c_[np.zeros_like(a),a,b]].astype('<f4');R=Rotation.from_euler('xyz',[.03,-.04,.2]).as_matrix();t=np.array([10.,20.,2.]);M=(S@R.T+t).astype('<f4')
  def write(name,P):
   p=cls.root/name
   with p.open('wb') as f:f.write(np.array([len(P)],'<u8').tobytes());f.write(P.tobytes())
   return str(p)
  cls.src=write('source',S);cls.target=write('target',M);cls.truth=np.eye(4);cls.truth[:3,:3]=R;cls.truth[:3,3]=t;seed=cls.truth.copy();seed[:3,3]+=[.02,-.02,.01];q=dict(case_id='synthetic',target_path=cls.target,target_sha='SYNTHETIC',source_path=cls.src,seed_common=seed.tolist(),roi=[True,1,*t,0,0,0,0,-10,30,120,1],curvature_probe=True);cls.proc=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
  def call(v):cls.proc.stdin.write(json.dumps(v)+'\n');cls.proc.stdin.flush();return json.loads(cls.proc.stdout.readline())
  cls.good=call(q);q['roi']=[True,1,-1000,-1000,0,0,0,0,0,-10,30,1,2];cls.bad=call(q)
 @classmethod
 def tearDownClass(cls):
  if hasattr(cls,'proc'):cls.proc.stdin.close();cls.proc.wait(timeout=10);cls.proc.stdout.close();cls.proc.stderr.close();cls.temp.cleanup()
 def test_roi_accepts_real_points_and_rejects_outside(self):
  self.assertTrue(self.good['candidate_valid']);self.assertGreater(self.good['pairs'],1000);self.assertFalse(self.bad['candidate_valid']);self.assertEqual(self.bad['pairs'],0)
 def test_known_transform_recovered(self):
  self.assertTrue(self.good['converged']);np.testing.assert_allclose(np.array(self.good['final']),self.truth,atol=2e-4)
 def test_final_full_information_and_gradient(self):
  H=np.array(self.good['H_right_rotation_translation']);np.testing.assert_allclose(H,H.T,atol=1e-8);self.assertGreater(np.linalg.eigvalsh(H)[0],0)
  for q in self.good['frozen_factor_curvature']:
   self.assertAlmostEqual((q['plus']-q['minus'])/(2*q['step']),q['gradient'],delta=1e-5)
