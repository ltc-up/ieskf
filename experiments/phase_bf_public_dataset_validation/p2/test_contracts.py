"""Small non-CUDA contract tests; no registrations or filter updates."""
import importlib.util,unittest
from pathlib import Path
import numpy as np

def module(name,file):
 spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(file));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
run=module('p2_run','run_five.py');analysis=module('p2_analysis','analyze_five.py')
class Contracts(unittest.TestCase):
 def test_exact_frame_scope(self):self.assertEqual(run.FRAMES,(60,64,65,66,76))
 def test_missing_wrapper_anchor_rejected(self):
  with self.assertRaises(ValueError):run.patch_wrapper('not the qualified wrapper')
 def test_repeated_wrapper_anchor_rejected(self):
  with self.assertRaises(ValueError):run.patch_wrapper('auto j=J::parse(line);auto j=J::parse(line);')
 def test_intrinsic_rotation_chart(self):
  a,b,c=.3,-.2,1.1
  Rx=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]);Ry=np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]]);Rz=np.array([[np.cos(c),-np.sin(c),0],[np.sin(c),np.cos(c),0],[0,0,1]])
  T=analysis.matrix(np.array([1.,2.,3.,a,b,c]));np.testing.assert_allclose(T[:3,:3],Rx@Ry@Rz,atol=1e-14);np.testing.assert_array_equal(T[:3,3],[1,2,3])
 def test_double_float_export_not_false_failure(self):
  p=np.array([80.0000026658,2.,3.,.01,.02,.03]);T=analysis.matrix(p);out=T.astype(np.float32).astype(float)
  self.assertGreater(abs(T[0,3]-out[0,3]),1e-6);np.testing.assert_array_equal(T[:3,3].astype(np.float32),out[:3,3].astype(np.float32))
if __name__=='__main__':unittest.main()
