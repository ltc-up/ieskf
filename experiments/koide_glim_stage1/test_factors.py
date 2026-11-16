import numpy as np
from scipy.spatial.transform import Rotation
from factor_diagnostics import *
def perturb(x):
 T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix();T[:3,3]=x[3:];return T
def test_all_12_joint_jacobian_columns():
 p=np.array([2.,-.3,1.]);T=perturb(np.array([.2,-.1,.3,1.,2.,-.4]));q=transform(T,p);J=np.c_[-hat(q),np.eye(3),T[:3,:3]@hat(p),-T[:3,:3]]
 for j in range(12):
  d=np.zeros(6);d[j%6]=1e-6
  def residual(sign):return -transform((np.linalg.inv(perturb(sign*d))@T if j<6 else T@perturb(sign*d)),p)
  assert np.max(abs((residual(1)-residual(-1))/2e-6-J[:,j]))<1e-8
def test_mean_covariance_not_scatter():
 p=np.array([[.01,0,0],[.2,0,0]]);C=np.array([np.eye(3),np.eye(3)*2]);v=Voxels(p,C,1);assert np.allclose(v.mean[0],[.105,0,0]);assert np.allclose(v.cov[0],np.eye(3)*1.5)
def test_joint_gauge_and_finite_information():
 rng=np.random.default_rng(9);p=rng.normal(size=(150,3));C=covariances(p);v=Voxels(p,C,1);a=evaluate(p,C,v,np.eye(4));H=np.array(a['H_joint']);assert np.max(abs(H-H.T))<1e-8;assert (np.abs(np.linalg.eigvalsh(H))<1e-6).sum()>=6;assert np.linalg.eigvalsh(H).min()>-1e-6
def test_voxel_floor_negative_coordinates():
 v=Voxels(np.array([[-.1,0,0]]),np.array([np.eye(3)]),1);assert v.overlap(np.array([[-.8,0,0]]))[0];assert not v.overlap(np.array([[.1,0,0]]))[0]
def test_motion_uses_no_future_or_map():
 I=[dict(header_ns=0,recorder_ns=0,omega=[0,0,0]),dict(header_ns=20_000_000,recorder_ns=20_000_000,omega=[0,0,10])];W=[dict(header_ns=0,recorder_ns=0,speed=10.)];T=motion(1,10_000_001,I,W,np.eye(3));assert np.allclose(T[:3,3],[.1,0,0]);assert np.allclose(T[:3,:3],np.eye(3))

def test_stale_and_future_anchor_rejected():
 from existing_15d_prediction import require_fresh_anchor
 import pytest
 require_fresh_anchor(30_000_000,20_000_000)
 with pytest.raises(ValueError):require_fresh_anchor(200_000_000,0)
 with pytest.raises(ValueError):require_fresh_anchor(0,1)
