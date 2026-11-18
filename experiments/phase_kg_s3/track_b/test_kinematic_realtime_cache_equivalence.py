import unittest
import numpy as np
from kinematic_realtime_support import MotionStore as Original
from kinematic_realtime_support_v2 import MotionStore as Cached
from kinematic_predictor import HeaderCoverageAudit


class CacheEquivalenceTests(unittest.TestCase):
    def test_late_duplicate_held_gap_and_prune_match_exactly(self):
        audit=HeaderCoverageAudit([0,10,20,30,40],[0,20,40],0,100,'toy');a=Original(0,audit);b=Cached(0,audit)
        events=[dict(kind='imu',header=0,available=0,id=1,omega=[.2,.3,.4]),dict(kind='wheel',header=0,available=0,id=2,speed=3.,variance=0.),dict(kind='imu',header=20,available=21,id=3,omega=[.3,.2,.1]),dict(kind='imu',header=10,available=22,id=4,omega=[-.1,.4,.7]),dict(kind='wheel',header=20,available=23,id=5,speed=4.,variance=.1)]
        for e in events:
            a.append(e);b.append(e)
            for target in [5,15,25,40_000_000]:
                self.assertEqual(a.predictor().query(0,target,23,0),b.predictor().query(0,target,23,0))
        a.append(events[-1]);b.append(events[-1]);self.assertEqual(a.predictor().query(0,25,23,0),b.predictor().query(0,25,23,0))
        a.prune(15);b.prune(15);self.assertEqual(a.predictor().query(15,25,23,0),b.predictor().query(15,25,23,0))
    def test_cached_snapshots_cannot_mutate_store(self):
        b=Cached(0);b.append(dict(kind='imu',header=0,available=0,id=1,omega=[0,0,0]));p=b.predictor();p.samples['imu'].clear();p.identities.clear();p.bias=np.ones(3)
        q=b.predictor();self.assertEqual(len(q.samples['imu']),1);self.assertEqual(q.bias.tolist(),[0,0,0])
    def test_changed_identity_error_still_occurs_at_snapshot(self):
        for cls in [Original,Cached]:
            b=cls(0);b.append(dict(kind='imu',header=0,available=0,id=1,omega=[0,0,0]));b.append(dict(kind='imu',header=0,available=0,id=1,omega=[1,0,0]))
            with self.assertRaisesRegex(ValueError,'MOTION_IDENTITY_CHANGED'):b.predictor()

if __name__=='__main__':unittest.main()
