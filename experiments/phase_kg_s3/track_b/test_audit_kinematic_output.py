import copy
import unittest
import numpy as np
from audit_kinematic_output import audit


def fixture():
    I=np.eye(4);D=I.copy();D[0,3]=.1
    events=[];frames=[]
    for i in range(2):
        ref=(i+1)*10_000_000;fid=str(i+1);original=ref+2
        U=I.copy();U[0,3]=.1*(i+1)
        z=dict(target='PREVIOUS',predictor='KINEMATIC_BK',O_T_base=I.tolist() if i==0 else D.tolist(),source_identity='source',accepted=False,status='INITIALIZED' if i==0 else 'PREDICTION_ONLY')
        if i:z.update(predicted_O_T_base=D.tolist(),targets=[dict(frame_id='1',age_ns=10_000_000,O_T_base=I.tolist())])
        segment=dict(start_ns=ref-10_000_000,end_ns=ref,imu_header_ns=ref-10_000_000,wheel_header_ns=ref-10_000_000,imu_available_ns=ref-10_000_000,wheel_available_ns=ref-10_000_000)
        events.append(dict(status='COMMITTED',reason='valid',frame_id=fid,ref_ns=ref,source_identity='source',original_cloud_available_ns=original,query_available_ns=original,delay_ns=0,interval=dict(status='VALID',query_available_ns=original,bias_base_rad_s=[0.,0.,0.],body_lateral_vertical_zero=True,prev_ref_ns=ref-10_000_000,segments=[segment],delta_T=D.tolist()),U_kin=U.tolist(),H_init=I.tolist() if i==0 else None,variants={'KINEMATIC_BK__PREVIOUS':z}))
        frames.append(dict(ref_ns=ref,available_ns=original,source_payload_sha256='source',B=dict(frame_id=fid)))
    return events,frames


class TestAudit(unittest.TestCase):
    def test_valid_chain(self):
        e,f=fixture();self.assertEqual(audit(e,f)['violations'],[])
    def test_global_reset_detected(self):
        e,f=fixture();e[1]['U_kin'][0][3]=10
        self.assertTrue(any('only integrates' in z['rule'] for z in audit(e,f)['violations']))
    def test_future_available_and_held_detected(self):
        e,f=fixture();s=e[1]['interval']['segments'][0];s['imu_available_ns']=30_000_000;s['wheel_header_ns']=-100_000_000
        r=audit(e,f);self.assertTrue(any('no future available' in z['rule'] for z in r['violations']));self.assertTrue(any('held limit' in z['rule'] for z in r['violations']))
    def test_delay_does_not_advance(self):
        e,f=fixture();delay=dict(e[1],status='DELAY',delta_T=None);e.insert(1,delay)
        r=audit(e,f);self.assertEqual(r['violations'],[]);self.assertEqual(r['delayed_frames'],1);self.assertEqual(r['committed'],2)
    def test_live_buffered_source_join_deferred(self):
        e,f=fixture();r=audit(e,f[:1]);self.assertEqual(r["violations"],[]);self.assertEqual(r["deferred_source_joins_buffered_prefix"],["2"])
    def test_current_source_change_detected(self):
        e,f=fixture();e[1]['source_identity']='other'
        self.assertTrue(any('same immutable current' in z['rule'] for z in audit(e,f)['violations']))

if __name__=='__main__':unittest.main()
