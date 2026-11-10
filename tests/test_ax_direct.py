import unittest,os,tempfile,subprocess,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State,inject
from geometry_ieskf.direct_iterated import DirectIterated,GeometryClient,base_pose,pose_chart
class Chart(unittest.TestCase):
 def test_chart_finite_difference(self):
  x=State(np.array([2.,3.,4.]),np.zeros(3),Rotation.from_rotvec([.2,-.1,.3]).as_matrix(),np.zeros(3),np.zeros(3));l=np.array([1.,.2,.4]);z=np.zeros(15);z[6:9]=[.1,.02,-.03];y=inject(x,z);T=base_pose(y,l);A=pose_chart(y,l,z);D=np.zeros((6,15));eps=1e-6
  for k in range(15):
   e=np.eye(15)[k]*eps;tp=base_pose(inject(x,z+e),l);tm=base_pose(inject(x,z-e),l);D[:3,k]=(Rotation.from_matrix(T[:3,:3].T@tp[:3,:3]).as_rotvec()-Rotation.from_matrix(T[:3,:3].T@tm[:3,:3]).as_rotvec())/(2*eps);D[3:,k]=T[:3,:3].T@(tp[:3,3]-tm[:3,3])/(2*eps)
  np.testing.assert_allclose(A,D,atol=6e-10)
 def test_single_prior_and_atomicity(self):
  class Client:
   def call(self,q):
    T=np.array(q['seed_common']);H=np.zeros((6,6));H[3:,3:]=np.eye(3)*4;g=np.r_[np.zeros(3),4*(T[:3,3]-np.array([.1,0,0]))];return dict(candidate_valid=True,H=H.tolist(),gradient=g.tolist(),cost=2*sum((T[:3,3]-[.1,0,0])**2),factors=100,groups=3,correspondence_changes=0)
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));P=np.eye(15);P[0,3]=P[3,0]=.3
  obs=DirectIterated(np.zeros(3),Client(),{});y,Q,r=obs.correct(x,P);self.assertTrue(r['accepted']);self.assertEqual(r['covariance_updates'],1);self.assertAlmostEqual(y.p[0],.08);self.assertAlmostEqual(Q[0,0],.2);self.assertAlmostEqual(y.v[0],.024);self.assertGreater(np.linalg.eigvalsh(Q).min(),0);self.assertLess(r['Joseph_error'],1e-12)
  obs.max_iterations=1;y,Q,r=obs.correct(x,P);self.assertFalse(r['accepted']);np.testing.assert_array_equal(Q,P);np.testing.assert_array_equal(y.p,x.p)
@unittest.skipUnless(os.environ.get('AX_BINARY'),'set AX_BINARY for executable geometric tests')
class RealService(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.proc=subprocess.Popen([os.environ['AX_BINARY']],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True);self.client=GeometryClient(self.proc)
 def tearDown(self):self.proc.stdin.close();self.proc.wait();self.proc.stdout.close();self.tmp.cleanup()
 def cloud(self,name,p):
  f=self.root/name
  with f.open('wb') as s:np.array([len(p)],'<u8').tofile(s);np.array(p,'<f4').tofile(s)
  return str(f)
 def request(self,p,target=None,identity='map'):
  path=self.cloud('target'+identity,p if target is None else target);src=self.cloud('src'+identity,p);return dict(source_path=src,target_path=path,target_sha=identity,seed_common=np.eye(4).tolist(),roi=[False,1,0,0,0,0,0,0,0,-100,100,120,1])
 def test_plane_null_duplicate_and_frozen_derivative(self):
  u=np.linspace(-4,4,21);p=np.array([[a,b,0] for a in u for b in u]);q=self.request(p);a=self.client.call(q);self.assertTrue(a['candidate_valid']);H=np.array(a['H']);self.assertEqual(np.linalg.matrix_rank(H,tol=1e-6),3);self.assertLess(np.linalg.norm(H[:,[2,3,4]]),1e-9);self.assertGreater(np.array(a['raw_gicp_H'])[3,3],1)
  dup=self.request(np.repeat(p,3,axis=0),np.repeat(p,2,axis=0),'dup');b=self.client.call(dup);np.testing.assert_allclose(a['H'],b['H'],rtol=1e-8,atol=1e-8)
  # Restore and perturb frozen associations; compare condensed cost gradient.
  self.client.call(q);T=np.eye(4);T[2,3]=.05;mid=self.client.call(q|dict(op='evaluate',seed_common=T.tolist()));eps=1e-5
  for k in [0,1,5]:
   E=np.eye(4);d=np.zeros(3)
   if k<3:d[k]=eps;E[:3,:3]=Rotation.from_rotvec(d).as_matrix()
   else:E[2,3]=eps
   pp=self.client.call(q|dict(op='evaluate',seed_common=(T@E).tolist()));mm=self.client.call(q|dict(op='evaluate',seed_common=(T@np.linalg.inv(E)).tolist()));self.assertAlmostEqual((pp['cost']-mm['cost'])/(2*eps),np.array(mid['gradient'])[k,0],places=5)
 def test_shared_frame_frozen_gradient_and_budget(self):
  p=np.array([[a,b,0] for a in np.linspace(-4,4,21) for b in np.linspace(-4,4,21)]);q=self.request(p)|dict(shared_frame=True);r=self.client.call(q);H=np.array(r['H']);self.assertLess(H[5,5],4.000001);T=np.eye(4);T[2,3]=.04;v=self.client.call(q|dict(op='evaluate',seed_common=T.tolist()));plus=T.copy();minus=T.copy();plus[2,3]+=1e-5;minus[2,3]-=1e-5;ep=self.client.call(q|dict(op='evaluate',seed_common=plus.tolist()))['cost'];em=self.client.call(q|dict(op='evaluate',seed_common=minus.tolist()))['cost'];self.assertAlmostEqual((ep-em)/2e-5,np.array(v['gradient'])[5,0],places=6)
 def test_actual_cpp_shared_blocks_against_dense(self):
  u=np.linspace(-4,4,15);p=np.array([[a,b,0] for a in u for b in u],dtype=np.float32).astype(float);q=self.request(p);r=self.client.call(q);J=np.c_[p[:,1],-p[:,0],np.zeros((len(p),3)),np.ones(len(p))];R=np.eye(len(p))*.05**2;keys=np.floor(p/5).astype(int);C=np.diag([.25**2]*3+[np.deg2rad(1.)**2]*3)
  for key in np.unique(keys,axis=0):
   ix=np.flatnonzero(np.all(keys==key,axis=1));v=p[ix]-(key+.5)*5;B=np.c_[np.zeros((len(ix),2)),np.ones(len(ix)),v[:,1],-v[:,0],np.zeros(len(ix))];R[np.ix_(ix,ix)]+=B@C@B.T
  np.testing.assert_allclose(r['H'],J.T@np.linalg.solve(R,J),atol=1e-7,rtol=1e-9)
 def test_two_three_planes_rank(self):
  u=np.linspace(-3,3,13);plane=lambda k:np.array([[a,b,0] for a in u for b in u])[:,np.roll(np.arange(3),k)]
  for n,rank in [(2,5),(3,6)]:
   p=np.concatenate([plane(k)+np.eye(3)[k]*8 for k in range(n)]);r=self.client.call(self.request(p,identity=str(n)));self.assertEqual(np.linalg.matrix_rank(np.array(r['H']),tol=1e-5),rank)

class SharedJoint(unittest.TestCase):
 def test_nonlinear_common_marginal(self):
  class Client:
   def call(self,q):
    T=np.array(q['seed_common']);J=np.zeros((3,6));J[:,3:]=T[:3,:3];r=T[:3,3]-[.1,0,0];return dict(candidate_valid=True,H=(J.T@J*4).tolist(),gradient=(J.T@r*4).tolist(),cost=float(2*r@r),factors=100,groups=1,correspondence_changes=0)
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));y,Q,r=DirectIterated(np.zeros(3),Client(),{'shared_frame':True}).correct(x,np.eye(15));self.assertTrue(r['accepted']);self.assertEqual(r['joint_dimension'],21);self.assertAlmostEqual(y.p[0],.1/1.5,places=6);self.assertAlmostEqual(Q[0,0],1/3,places=6);self.assertEqual(r['covariance_updates'],1)
 def test_shared_dense_schur(self):
  from geometry_ieskf.surface_observation import Plane,SharedPlaneNoise,tangent_basis
  rng=np.random.default_rng(4);n=40;J=rng.normal(size=(n,3));plane=Plane('p',np.array([0,0,1.]),np.zeros(3),tangent_basis([0,0,1.]),np.diag([.01,.01,.1]));noise=SharedPlaneNoise(['p']*n,J,{'p':plane},.05**2);A=rng.normal(size=(n,15));r=rng.normal(size=n);dense=np.linalg.solve(noise.dense(),A);np.testing.assert_allclose(noise.solve(A),dense,rtol=1e-9,atol=1e-8)

class JointJacobianAndDelay(unittest.TestCase):
 def test_augmented_right_se3_finite_difference(self):
  from geometry_ieskf.direct_iterated import augmented_chart
  x=State(np.array([1.,2.,3.]),np.ones(3),Rotation.from_rotvec([.2,-.3,.4]).as_matrix(),np.zeros(3),np.zeros(3));lever=np.array([1.,.2,-.3]);z=np.linspace(-.04,.04,21)
  def T(v):
   q=base_pose(inject(x,v[:15]),lever);E=np.eye(4);E[:3,:3]=Rotation.from_rotvec(v[15:18]).as_matrix();E[:3,3]=v[18:];return q@E
  y=inject(x,z[:15]);A=augmented_chart(y,lever,z);base=T(z);D=np.zeros((6,21));eps=1e-6
  for j in range(21):
   e=np.eye(21)[j]*eps;plus=T(z+e);minus=T(z-e);D[:3,j]=(Rotation.from_matrix(base[:3,:3].T@plus[:3,:3]).as_rotvec()-Rotation.from_matrix(base[:3,:3].T@minus[:3,:3]).as_rotvec())/(2*eps);D[3:,j]=base[:3,:3].T@(plus[:3,3]-minus[:3,3])/(2*eps)
  np.testing.assert_allclose(A,D,atol=5e-10)
 def test_delayed_repropagation_and_duplicate(self):
  from ndt_global.ar_replay import AuditedPoseReplay
  class Client:
   def call(self,q):
    T=np.array(q['seed_common']);J=np.zeros((3,6));J[:,3:]=T[:3,:3];r=T[:3,3]-[.1,0,0];return dict(candidate_valid=True,H=(J.T@J*4).tolist(),gradient=(J.T@r*4).tolist(),cost=float(2*r@r),factors=100,groups=1,correspondence_changes=0)
  import copy
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),1);P=np.eye(15);imu=dict(header_ns=1,omega=np.zeros(3),force=np.array([0,0,9.80665]));r1=AuditedPoseReplay(x,P,imu,np.zeros(3));r2=AuditedPoseReplay(x,P,imu,np.zeros(3));obs=DirectIterated(np.zeros(3),Client(),{'shared_frame':True});e=dict(kind='ndt',header_ns=5_000_001,message_id=99,observer=obs,pose=None,R=None,correlation_group='same_cloud')
  for k in range(1,10):
   m=dict(kind='imu',header_ns=k*1_000_000+1,message_id=k,omega=np.zeros(3),force=np.array([0,0,9.80665]));r1.ingest(copy.deepcopy(m));r2.ingest(copy.deepcopy(m))
   if k==5:r1.ingest(copy.copy(e))
  r2.ingest(e);self.assertTrue(e['last_update']['accepted']);a,A,_=r1.current();b,B,_=r2.current();np.testing.assert_allclose(a.p,b.p,atol=1e-13);np.testing.assert_allclose(A,B,atol=1e-13);self.assertEqual(r2.ingest(e),'DUPLICATE');np.testing.assert_array_equal(r2.current()[1],B)
