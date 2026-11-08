import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,PoseHistory
from ndt_global.global_manager import GlobalManager,PositionObservation
from ndt_global.geodesy import enu,ecef,enu_from_ned

class GlobalTests(unittest.TestCase):
    def setUp(self):
        self.c=Contract('m','e','o');self.h=PoseHistory(self.c,max_gap_ns=1_000_000_000)
        for k in range(5):self.h.add(k*100_000_000,np.eye(4),self.c)
        self.g=GlobalManager(self.h)
    def pos(self,t=0,**kw):
        d=dict(stamp_ns=t,source_id='gps',correlation_group=str(t),position_base_map=np.array([1.,2.,3.]),contract=self.c,fix_valid=True,datum_qualified=True,reference_qualified=True,clock_qualified=True);d.update(kw);return PositionObservation(**d)
    def test_partial_no_yaw_and_recovery(self):
        R=Rotation.from_euler('z',.4).as_matrix();self.g.C[:3,:3]=R
        self.assertTrue(self.g.position(self.pos(),0)['accepted']);self.assertEqual(self.g.status(0)['state'],'RECOVERING')
        self.g.position(self.pos(100_000_000),100_000_000)
        self.assertEqual(self.g.status(100_000_000)['state'],'PARTIAL');np.testing.assert_array_equal(self.g.C[:3,:3],R)
        self.assertIn('yaw',self.g.status(100_000_000)['unresolved'])
        self.assertEqual(self.g.status(800_000_000)['state'],'COASTING')
        self.assertEqual(self.g.status(3_000_000_000)['state'],'UNAVAILABLE')
        self.h.add(3_000_000_000,np.eye(4),self.c)
        self.assertTrue(self.g.position(self.pos(3_000_000_000),3_000_000_000)['accepted'])
        self.assertEqual(self.g.status(3_000_000_000)['state'],'RECOVERING')
    def test_qualified_initial_orientation_not_gps_heading(self):
        g=GlobalManager(self.h,orientation_qualified=True)
        g.position(self.pos(),0);g.position(self.pos(100_000_000),100_000_000)
        self.assertEqual(g.status(100_000_000)['state'],'GLOBAL_TRACKING');self.assertFalse(g.status(100_000_000)['trusted_global'])
    def test_ambiguity_preserved_and_bounded_pruning(self):
        A=np.eye(4);A[:3,3]=[1,2,3];B=A.copy();B[0,3]=6
        self.g.ndt([A,B],'AMBIGUOUS',0,'scan',0,self.c)
        self.assertEqual(self.g.status(0)['state'],'AMBIGUOUS')
        self.g.position(self.pos(),0);self.assertEqual(len(self.g.pending),2)
        g=self.pos(100_000_000,error_bounds_m=np.array([.2,.2,.2]),bounds_qualified=True,source_independent_for_selection=True)
        self.g.position(g,100_000_000);self.assertEqual(len(self.g.pending),1)
        self.assertEqual(self.g.status(100_000_000)['state'],'PARTIAL')
    def test_atomic_bad_bounds_and_contract(self):
        self.g.ndt([np.eye(4),np.eye(4)],'AMBIGUOUS',0,'s',0,self.c);C=self.g.C.copy()
        event=self.g.position(self.pos(error_bounds_m=[-1,1,1],bounds_qualified=True,source_independent_for_selection=True),0)
        self.assertEqual(event['reason'],'INVALID_BOUNDS');self.assertIsNone(self.g.anchor_ns);np.testing.assert_array_equal(C,self.g.C)
        self.assertFalse(self.g.position(self.pos(contract=replace(self.c,epoch='other')),0)['accepted'])
    def test_unqualified_gnss_received_not_missing(self):
        d=self.g.position(self.pos(fix_valid=False),0)
        self.assertFalse(d['accepted']);self.assertTrue(self.g.status(0)['gnss']['received']);self.assertEqual(self.g.status(0)['state'],'UNAVAILABLE')
    def test_duplicate_rollback_stale(self):
        self.g.position(self.pos(),0);self.assertEqual(self.g.position(self.pos(),0)['reason'],'DUPLICATE')
        self.assertEqual(self.g.position(self.pos(1),600_000_000)['reason'],'STALE')
        with self.assertRaises(ValueError):self.g.status(1)
    def test_local_and_full_covariance_have_no_write_path(self):
        before=[x.copy() for x in self.h.poses];P=np.eye(15);old=P.copy()
        self.g.ndt([np.eye(4)],'UNKNOWN',0,'s',0,self.c);self.g.position(self.pos(),0)
        for a,b in zip(before,self.h.poses):np.testing.assert_array_equal(a,b)
        np.testing.assert_array_equal(P,old)
    def test_geodesy_and_ned(self):
        np.testing.assert_allclose(ecef([0,0,0]),[6378137,0,0],atol=1e-8)
        np.testing.assert_allclose(enu([0,0,10],[0,0,0]),[0,0,10],atol=1e-8)
        np.testing.assert_allclose(enu([0,.00001,0],[0,0,0])[0],1.1131949,atol=1e-6)
        np.testing.assert_array_equal(enu_from_ned([1,2,3]),[2,1,-3])
    def test_dimensional_expiry(self):
        self.g.position(self.pos(),0)
        self.h.add(700_000_000,np.eye(4),self.c)
        self.g.position(self.pos(700_000_000,dimensions=(0,1)),700_000_000)
        self.assertIn('z',self.g.status(700_000_000)['unresolved'])
    def test_temporal_hypothesis_transport(self):
        self.h.poses[1][0,3]=2
        A=np.eye(4);B=np.eye(4);B[0,3]=5
        self.g.ndt([A,B],'AMBIGUOUS',0,'s',0,self.c)
        p=self.pos(100_000_000,position_base_map=[2.,0.,0.],error_bounds_m=[.1,.1,.1],bounds_qualified=True,source_independent_for_selection=True)
        self.assertTrue(self.g.position(p,100_000_000)['accepted']);self.assertEqual(len(self.g.pending),1)
        self.assertAlmostEqual(self.g.pending[0][0,3],2)
if __name__=='__main__':unittest.main()
