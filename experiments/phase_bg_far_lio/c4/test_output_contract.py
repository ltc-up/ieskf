"""Small numerical/output contract checks; no recorded sensor input."""
import unittest, threading
import numpy as np
from high_rate_output import SnapshotStore
class X:
 def __init__(self,t=100_000_000):
  self.t=t;self.p=np.zeros(3);self.v=np.zeros(3);self.R=np.eye(3);self.bg=np.zeros(3);self.ba=np.zeros(3)
class TestSnapshot(unittest.TestCase):
 def setUp(self):self.s=SnapshotStore();self.x=X();self.P=np.eye(15)
 def put(self):self.s.commit(self.x,self.P,self.x.t)
 def test_uninitialized(self):self.assertEqual(self.s.packet(0,0,0)['health'],'UNINITIALIZED')
 def test_current(self):self.put();self.assertTrue(self.s.packet(self.x.t,0,0)['valid'])
 def test_stale(self):self.put();self.assertEqual(self.s.packet(self.x.t+20_000_001,0,0)['health'],'STALE')
 def test_future(self):self.put();self.assertFalse(self.s.packet(self.x.t-1,0,0)['valid'])
 def test_stamp_preserved(self):self.put();self.assertEqual(self.s.packet(self.x.t+100,0,0)['state_time_ns'],self.x.t)
 def test_age(self):self.put();self.assertEqual(self.s.packet(self.x.t+123,0,0)['state_age_ns'],123)
 def test_reader_copy(self):self.put();p=self.s.packet(self.x.t,0,0);p['P'][0][0]=99;self.assertEqual(self.s.packet(self.x.t,0,0)['P'][0][0],1)
 def test_owner_copy(self):self.put();self.P[0,0]=4;self.x.p[0]=5;self.assertEqual(self.s.packet(self.x.t,0,0)['position_map'][0],0)
 def test_no_information_repeat(self):self.put();a=self.s.packet(self.x.t,0,0);[self.s.packet(self.x.t,0,i) for i in range(100)];self.assertEqual(a['P'],self.s.packet(self.x.t,0,0)['P']);self.assertEqual(self.s.revision,1)
 def test_atomic(self):
  self.put();errors=[]
  def writer():
   for i in range(100):
    x=X(self.x.t+i);x.p[:]=i;self.s.commit(x,np.eye(15)*(i+1),x.t)
  th=threading.Thread(target=writer);th.start()
  for i in range(100):
   p=self.s.packet(self.x.t+100,0,i)
   if p['P'][0][0]!=p['position_map'][0]+1:errors.append(p)
  th.join();self.assertFalse(errors)
 def test_backward_commit(self):
  self.put();self.x.t-=1
  with self.assertRaises(ValueError):self.s.commit(self.x,self.P,self.x.t)
  self.assertEqual(self.s.revision,1)
 def test_bad_shape(self):
  with self.assertRaises(ValueError):self.s.commit(self.x,np.eye(6),self.x.t)
 def test_bad_mean(self):
  self.x.p[0]=np.nan
  with self.assertRaises(ValueError):self.put()
 def test_future_imu(self):
  with self.assertRaises(ValueError):self.s.commit(self.x,self.P,self.x.t+1)
 def test_same_time_new_revision(self):
  self.put();self.put();self.assertEqual(self.s.revision,2)
 def test_bad_P(self):
  for kind in ['nan','negative','asymmetric']:
   p=self.P.copy()
   if kind=='nan':p[0,0]=np.nan
   elif kind=='negative':p[0,0]=-1
   else:p[0,1]=1
   with self.assertRaises(ValueError):self.s.commit(self.x,p,self.x.t)
if __name__=='__main__':unittest.main()
