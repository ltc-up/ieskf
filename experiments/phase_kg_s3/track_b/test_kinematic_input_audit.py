import unittest
import numpy as np
from kinematic_input_audit import verify_frame_binding

class FrameBindingTests(unittest.TestCase):
 def test_raw_frames_and_exact_static_binding(self):
  metadata={'original_frame_ids':{'imu':['imu_bosch'],'wheel':['base_link']}}
  audit={'tf_static':[{'child':'imu_bosch','parent':'base_link','quaternion':[0,0,0,1],'translation':[0,0,0]}]}
  r=verify_frame_binding(metadata,audit,np.eye(4));self.assertEqual(r['gyro_rotation_count'],1)
  metadata['original_frame_ids']['wheel']=['imu_bosch']
  with self.assertRaisesRegex(ValueError,'FRAME_CONTRACT'):verify_frame_binding(metadata,audit,np.eye(4))
 def test_wrong_imu_extrinsic_does_not_silently_adapt(self):
  m={'original_frame_ids':{'imu':['imu_bosch'],'wheel':['base_link']}}
  a={'tf_static':[{'child':'imu_bosch','parent':'base_link','quaternion':[0,0,0,1],'translation':[1,0,0]}]}
  with self.assertRaisesRegex(ValueError,'BINDING'):verify_frame_binding(m,a,np.eye(4))

if __name__=='__main__':unittest.main()
