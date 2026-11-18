import copy,unittest
from types import SimpleNamespace
import numpy as np
from lifecycle_audit import LifecycleAudit

class Log:
    def __init__(self):self.rows=[]
    def put(self,row):self.rows.append(row)
class Observer:
    def __init__(self):self.calls=0;self.accepted=True
    def correct(self,x,P,T,R=None,projection=None):
        self.calls+=1
        return x,P,dict(accepted=self.accepted,reason='TEST',score_calls=self.calls)

class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.log=Log();self.audit=LifecycleAudit(self.log);self.inner=Observer();self.wrapper=self.audit.wrap(self.inner,'scan:1')
        self.x=SimpleNamespace(t=123,p=np.zeros(3),v=np.zeros(3),R=np.eye(3),bg=np.zeros(3),ba=np.zeros(3));self.P=np.eye(15)
    def test_passive_result_identity_and_unchanged_counters(self):
        y,Q,info=self.wrapper.correct(self.x,self.P,np.eye(4))
        self.assertIs(y,self.x);self.assertIs(Q,self.P);self.assertEqual(info['score_calls'],1)
        self.wrapper.correct(self.x,self.P,np.eye(4));self.assertEqual(len(self.log.rows),1)
    def test_changed_prior_logged_and_old_arrays_frozen(self):
        self.wrapper.correct(self.x,self.P,np.eye(4));self.x.p[0]=2
        self.wrapper.correct(self.x,self.P,np.eye(4))
        self.assertEqual(self.log.rows[0]['prior']['p'][0],0);self.assertEqual(self.log.rows[1]['revision'],1)
    def test_deepcopy_preserves_one_audit_sink(self):
        copied=copy.deepcopy(self.wrapper);self.assertIs(copied.audit,self.audit)
    def test_final_ledger_mismatch_detected_after_numerical_revocation(self):
        self.inner.accepted=False;self.wrapper.correct(self.x,self.P,np.eye(4))
        replay=SimpleNamespace(records={'scan:1':SimpleNamespace(status='ACCEPTED')})
        self.assertEqual(len(self.audit.finish(replay)['ledger_numerical_mismatches']),1)

if __name__=='__main__':unittest.main()
