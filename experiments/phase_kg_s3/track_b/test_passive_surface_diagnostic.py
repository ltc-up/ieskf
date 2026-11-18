import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from passive_surface_diagnostic import historical_base_diagnostic,PassiveMatcher,probe_class,SurfaceStatus
from glim_diagnostics import surface_mask
from local_odometry import transform,Scan,cloud_hash
from kinematic_predictor import IntervalKinematicPredictor


class PassiveSurfaceTests(unittest.TestCase):
    def test_sidecar_closure_is_independent(self):
        status=SurfaceStatus();status.record(dict(status='EVALUATED',diagnostic_cpu_ms=2.));status.record(dict(status='DIAGNOSTIC_ERROR',diagnostic_cpu_ms=1.))
        self.assertFalse(status.summary(True,False,2)['sidecar_complete']);self.assertFalse(status.summary(True,True,1)['sidecar_complete'])
        result=status.summary(True,True,2);self.assertTrue(result['sidecar_complete']);self.assertEqual(result['diagnostic_errors'],1);self.assertEqual(result['diagnostic_total_cpu_ms'],3.)
        self.assertFalse(SurfaceStatus().summary(False,False,0)['sidecar_complete'])
    def test_physical_gauge_invariant(self):
        rng=np.random.default_rng(4);source=rng.normal(size=(64,3));target=rng.normal(size=(70,3));n=source/np.linalg.norm(source,axis=1)[:,None]
        anchor=np.eye(4);anchor[:3,3]=[10,20,3];candidate=anchor.copy();candidate[:3,3]+=[.1,.2,.3]
        G=np.eye(4);G[:3,:3]=Rotation.from_euler('xyz',[.3,.2,-.5]).as_matrix();G[:3,3]=[-50,7,2]
        a=historical_base_diagnostic(source,transform(anchor,target),candidate,anchor,n)
        b=historical_base_diagnostic(source,transform(G@anchor,target),G@candidate,G@anchor,n)
        self.assertEqual(a['levels'],b['levels']);self.assertEqual(a['surface_pass_points'],b['surface_pass_points'])
        np.testing.assert_allclose(a['historical_T_current'],b['historical_T_current'],atol=1e-13)
    def test_wrong_O_origin_changes_ray(self):
        p=np.array([[1.,0,0]]);n=np.array([[-1.,0,0]]);anchor=np.eye(4);anchor[0,3]=-100
        a=historical_base_diagnostic(p,transform(anchor,p),anchor,anchor,n)
        self.assertEqual(a['surface_pass_points'],1);self.assertEqual(surface_mask(p,n,anchor).sum(),0)
    def test_fail_open_and_exact_original_object(self):
        calls=[];logs=[];result={'untouched':True}
        def matcher(r):calls.append(r);return result
        q=dict(source=np.ones((2,3)),target=np.ones((2,3)),initial_guess=np.eye(4))
        wrapper=PassiveMatcher(matcher,np.eye(4),logs.append,{});before=q['source'].copy();actual=wrapper(q)
        self.assertIs(actual,result);self.assertEqual(len(calls),1);self.assertEqual(logs[0]['status'],'DIAGNOSTIC_ERROR');np.testing.assert_array_equal(q['source'],before)
    def test_no_extra_registration_or_result_mutation(self):
        p=IntervalKinematicPredictor(0)
        for kind in ['imu','wheel']:p.ingest(dict(kind=kind,header_ns=0,available_ns=0,message_id=1,**({'omega_base':[0,0,0]} if kind=='imu' else {'speed_base':2.,'variance':0.})))
        a=np.random.default_rng(5).normal(size=(100,3));calls=[];logs=[];pairlogs=[];results=[]
        def matcher(r):
            calls.append(r);z=dict(O_T_base=r['initial_guess'].tolist(),candidate_valid=True,converged=True,target_valid_voxels=20,neighbor_points=100);results.append(z);return z
        chain=SimpleNamespace(previous=SimpleNamespace(scan=SimpleNamespace(ref_ns=0),O_T_base=np.eye(4)),O=np.eye(4),_target=lambda scan,seed:(a+1,[],[]))
        probe=probe_class(logs.append)(matcher,chain,p,pairlogs.append);scan=Scan.create('2',20_000_000,20_000_000,a,a,'h');probe.arm(scan,['case'])
        request=dict(source=scan.source,source_identity=scan.source_identity,target=a,target_identity=cloud_hash(a),initial_guess=np.eye(4),frame_id='2');r=probe(request);probe.end_frame()
        self.assertEqual(len(calls),4);self.assertEqual(len(logs),4);self.assertIs(r,results[0]);self.assertNotIn('validation',r);self.assertTrue(all(z['status']=='EVALUATED' for z in logs));self.assertTrue(all(c['source'] is scan.source for c in calls))

if __name__=='__main__':unittest.main()
