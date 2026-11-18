import unittest
import threading
import numpy as np
from kinematic_realtime_support import MotionStore,PoseStore,predict_publication


def event(kind,t,i,value,available=None):
    e=dict(kind=kind,header=t,available=t if available is None else available,id=i)
    if kind=='imu':e.update(omega=[0,0,value])
    else:e.update(speed=value,variance=0.)
    return e


class TestRealtimeSupport(unittest.TestCase):
    def fixture(self):
        m=MotionStore(0);m.append(event('imu',0,1,1.));m.append(event('wheel',0,2,10.))
        p=PoseStore();p.commit(0,np.eye(4),'first','source',[])
        return p,m
    def test_actual_twist_not_restamp(self):
        p,m=self.fixture();r=predict_publication(p,m,10_000_000,10_000_000)
        self.assertTrue(r['valid']);T=np.asarray(r['O_T_base']);self.assertAlmostEqual(T[0,3],10*np.sin(.01),places=12);self.assertAlmostEqual(T[1,3],10*(1-np.cos(.01)),places=12)
    def test_held_expired_no_fake_pose(self):
        p,m=self.fixture();r=predict_publication(p,m,26_000_000,26_000_000);self.assertFalse(r['valid']);self.assertNotIn('O_T_base',r)
    def test_snapshot_mutation_isolated(self):
        p,m=self.fixture();x=p.read();x['O_T_base'][0,3]=99
        self.assertEqual(p.read()['O_T_base'][0,3],0)
        q=m.predictor();q.bias=np.ones(3);self.assertEqual(m.predictor().bias.tolist(),[0,0,0])
    def test_future_available_ignored(self):
        p,m=self.fixture();m.append(event('imu',5_000_000,3,100.,available=20_000_000));r=predict_publication(p,m,10_000_000,10_000_000)
        self.assertAlmostEqual(np.arctan2(r['O_T_base'][1][0],r['O_T_base'][0][0]),.01,places=12)
    def test_monotonic_commit_and_input(self):
        p,m=self.fixture()
        with self.assertRaises(ValueError):p.commit(0,np.eye(4),'other','x',[])
        m.append(event('imu',1,3,0.,available=10))
        with self.assertRaises(ValueError):m.append(event('wheel',2,4,0.,available=9))
    def test_publisher_never_calls_gpu(self):
        p,m=self.fixture();result=[]
        th=threading.Thread(target=lambda:result.append(predict_publication(p,m,10_000_000,10_000_000)));th.start();th.join(1)
        self.assertFalse(th.is_alive());self.assertTrue(result[0]['valid'])
    def test_concurrent_commit_prune_retries_anchor(self):
        p,m=self.fixture();original=m.predictor;calls=[]
        def competing():
            calls.append(1)
            if len(calls)==1:
                m.append(event('imu',5_000_000,3,1.));m.append(event('wheel',5_000_000,4,10.))
                p.commit(5_000_000,np.eye(4),'next','source',[]);m.prune(5_000_000)
            return original()
        m.predictor=competing
        r=predict_publication(p,m,10_000_000,10_000_000)
        self.assertTrue(r['valid']);self.assertEqual(r['anchor_ref_ns'],5_000_000);self.assertEqual(r['revision'],2);self.assertEqual(len(calls),2)
        self.assertAlmostEqual(r['O_T_base'][0][3],10*np.sin(.005),places=12)
    def test_prune_keeps_held_prefix(self):
        p,m=self.fixture();m.append(event('imu',10_000_000,3,1.));m.prune(5_000_000)
        self.assertEqual(len(m.predictor().samples['imu']),2)

if __name__=='__main__':unittest.main()
