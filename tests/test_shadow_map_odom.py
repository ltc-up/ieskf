import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,Candidate,PoseHistory,ShadowCorrector,se3

class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.c=Contract('map-v1','ext-v1','epoch1');self.h=PoseHistory(self.c)
        A=np.eye(4);B=A.copy();B[0,3]=2;B[:3,:3]=Rotation.from_euler('z',.2).as_matrix()
        self.h.add(0,A,self.c);self.h.add(100_000_000,B,self.c)
        self.s=ShadowCorrector(self.h)
    def candidate(self, **kw):
        d=dict(stamp_ns=50_000_000,source_id='source',correlation_group='scan',pose=np.eye(4),contract=self.c,converged=True,backend_health='OK');d.update(kw);return Candidate(**d)
    def test_interpolation_and_composition(self):
        O=self.h.at(50_000_000);self.assertAlmostEqual(O[0,3],1)
        self.assertAlmostEqual(Rotation.from_matrix(O[:3,:3]).as_rotvec()[2],.1)
        c=self.candidate();self.assertTrue(self.s.consume(c,100_000_000)['accepted'])
        np.testing.assert_allclose(self.s.publish(c.stamp_ns)['pose'],c.pose,atol=1e-12)
        self.assertFalse(self.s.publish(c.stamp_ns)['trusted'])
    def test_duplicate_and_stale(self):
        c=self.candidate();self.s.consume(c,100_000_000)
        self.assertEqual(self.s.consume(c,100_000_000)['reason'],'DUPLICATE')
        self.assertEqual(self.s.consume(self.candidate(correlation_group='new'),1_000_000_000)['reason'],'STALE')
    def test_delayed_not_latest(self):
        c=self.candidate();self.s.consume(c,150_000_000)
        np.testing.assert_allclose(self.s.correction,np.linalg.inv(self.h.at(c.stamp_ns)))
        self.assertFalse(np.allclose(self.s.correction,np.linalg.inv(self.h.at(100_000_000))))
    def test_versions_and_unknown(self):
        for field in ('epoch','map_version','extrinsic_version','reference','map_frame','direction'):
            self.assertEqual(self.s.consume(self.candidate(contract=replace(self.c,**{field:'other'})),100_000_000)['reason'],'CONTRACT_MISMATCH')
        with self.assertRaises(ValueError): PoseHistory(Contract('UNKNOWN','x','e'))
    def test_bad_pose_and_missing_history(self):
        T=np.eye(4);T[0,0]=2
        self.assertEqual(self.s.consume(self.candidate(pose=T),100_000_000)['reason'],'INVALID_TRANSFORM')
        self.assertEqual(self.s.consume(self.candidate(stamp_ns=-1),100_000_000)['reason'],'NO_BRACKET')
        self.assertEqual(self.s.consume(self.candidate(stamp_ns=200_000_000),100_000_000)['reason'],'FUTURE')
    def test_local_immutable_wrong_candidate(self):
        before=[v.copy() for v in self.h.poses];T=np.eye(4);T[2,3]=100
        self.s.consume(self.candidate(pose=T),100_000_000)
        for a,b in zip(before,self.h.poses):np.testing.assert_array_equal(a,b)
        self.assertAlmostEqual(self.s.publish(50_000_000)['pose'][2,3],100)
    def test_clock_and_gap_and_bounds(self):
        with self.assertRaises(ValueError):self.h.add(0,np.eye(4),self.c)
        self.h.add(1_000_000_000,np.eye(4),self.c)
        with self.assertRaises(ValueError):self.h.at(500_000_000)
        for k in range(2,40):self.h.add(k*1_000_000_000,np.eye(4),self.c)
        self.assertLessEqual(len(self.h.stamps),4)
    def test_convergence_backend_and_rollback(self):
        self.assertEqual(self.s.consume(self.candidate(converged=None),100_000_000)['reason'],'CONVERGENCE_UNKNOWN')
        self.assertFalse(self.s.consume(self.candidate(backend_health='UNKNOWN'),100_000_000)['accepted'])
        self.s.consume(self.candidate(),100_000_000)
        self.assertEqual(self.s.consume(self.candidate(stamp_ns=0,correlation_group='old'),100_000_000)['reason'],'OBS_TIME_ROLLBACK')
if __name__=='__main__':unittest.main()
