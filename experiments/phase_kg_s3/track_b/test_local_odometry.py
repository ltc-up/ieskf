import unittest
import numpy as np
from local_odometry import *


def T(x=0, z=0, tilt=0):
    out=np.eye(4);out[0,3]=x;out[2,3]=z
    c,s=np.cos(tilt),np.sin(tilt);out[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
    return out


def scan(i, points=None):
    p=np.array([[0.,0,1],[1,0,1],[2,0,1]]) if points is None else np.array(points,dtype=float)
    return Scan.create(str(i),i*50_000_000,i*50_000_000+1,p,p[:1],'synthetic shared Deskew')


class Tests(unittest.TestCase):
    def test_failure_keeps_strict_previous(self):
        o=LocalOdometry();o.step(scan(1),T(),None,None)
        o.step(scan(2),T(1),lambda r: {'O_T_base':T(99).tolist()},lambda r,x:False)
        r=o.step(scan(3),T(2),lambda r: {'O_T_base':r['initial_guess'].tolist()},lambda r,x:True)
        self.assertEqual(r['targets'][0]['frame_id'],'2');self.assertTrue(r['targets'][0]['predicted_anchor'])
        np.testing.assert_allclose(o.O,T(2));np.testing.assert_allclose(o.U,T(2))

    def test_lidar_does_not_change_U(self):
        o=LocalOdometry();o.step(scan(1),T(),None,None)
        o.step(scan(2),T(1),lambda r: {'O_T_base':T(2).tolist()},lambda r,x:True)
        r=o.step(scan(3),T(2),lambda r: {'O_T_base':r['initial_guess'].tolist()},lambda r,x:True)
        np.testing.assert_allclose(r['predicted_O_T_base'],T(3));np.testing.assert_allclose(o.U,T(2))

    def test_tilted_history_crop_uses_O_vertical(self):
        o=LocalOdometry();s=scan(1,[[0,0,20],[20,0,0],[0,0,31]])
        o.step(s,T(),None,None);o.previous.O_T_base=T(tilt=np.pi/2)
        cloud,_,ids=o._target(scan(2),T())
        self.assertEqual(ids[0]['selected_ids_sha256'],hashlib.sha256(np.array([0,2],dtype=np.int64).tobytes()).hexdigest())
        self.assertEqual(len(cloud),2)

    def test_dense_not_source_rectangle(self):
        o=LocalOdometry();o.step(scan(1,[[0,80,0],[119,0,0],[121,0,0],[0,0,-11],[0,0,30]]),T(),None,None)
        cloud,_,ids=o._target(scan(2),T());self.assertEqual(ids[0]['selected_ids_sha256'],hashlib.sha256(np.array([0,1,4],dtype=np.int64).tobytes()).hexdigest())
        self.assertEqual(len(cloud),3)

    def test_source_immutable_and_identical(self):
        o=LocalOdometry();o.step(scan(1),T(),None,None);s=scan(2)
        def matcher(r):
            self.assertIs(r['source'],s.source)
            with self.assertRaises(ValueError):r['source'][0,0]=9
            return {'O_T_base':r['initial_guess'].tolist()}
        o.step(s,T(1),matcher,lambda r,x:True)

    def test_duplicate_and_out_of_order(self):
        o=LocalOdometry();o.step(scan(2),T(),None,None)
        for i in [2,1]:
            with self.assertRaises(ValueError):o.step(scan(i),T(),None,None)

    def test_rolling_and_keyframes_are_separate(self):
        for mode in ['B-M','B-K','B-P']:
            o=LocalOdometry(Config(mode=mode,max_frames=2))
            for i in range(1,6):
                r=o.step(scan(i),T(i),lambda q:{'O_T_base':q['initial_guess'].tolist()},lambda q,x:True)
            self.assertLessEqual(len(o.history),2)
            self.assertEqual(r['variant'],mode)
            if mode=='B-P':self.assertFalse(r['accepted'])

    def test_initial_tilt_keeps_gravity_axis(self):
        o=LocalOdometry();o.step(scan(1,[[0,0,20],[20,0,0]]),T(tilt=np.pi/3),None,None)
        np.testing.assert_allclose(o.O[:3,:3],T(tilt=np.pi/3)[:3,:3])
        _,_,ids=o._target(scan(2),T())
        self.assertEqual(ids[0]['selected_ids_sha256'],hashlib.sha256(np.array([0],dtype=np.int64).tobytes()).hexdigest())

    def test_noncommuting_increment(self):
        o=LocalOdometry();U0=T(2,tilt=.3);U1=T(4,tilt=.4)
        o.step(scan(1),U0,None,None)
        r=o.step(scan(2),U1,lambda q:{'O_T_base':q['initial_guess'].tolist()},lambda q,x:True)
        np.testing.assert_allclose(r['O_T_base'],o.O_T_U@U1,atol=1e-12)

if __name__=='__main__':unittest.main()
