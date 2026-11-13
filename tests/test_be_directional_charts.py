import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.directional_shadow import ndt_information,gicp_information

def test_ndt_euler_information_matches_physical_world_yaw_difference():
    e=np.array([.24,-.31,.52]);R=Rotation.from_euler('XYZ',e).as_matrix();t=np.array([.6,.8]);U=np.array([[.6,-.8],[.8,.6]]);L=10.
    A=np.arange(36,dtype=float).reshape(6,6)/50+np.eye(6);H=A.T@A
    eps=1e-6
    ep=Rotation.from_matrix(Rotation.from_rotvec([0,0,eps/L]).as_matrix()@R).as_euler('XYZ')
    em=Rotation.from_matrix(Rotation.from_rotvec([0,0,-eps/L]).as_matrix()@R).as_euler('XYZ')
    B=np.zeros((6,3));B[:2,:2]=U;B[3:,2]=(ep-em)/(2*eps)
    actual=np.asarray(ndt_information(-H,e,t,L)['information'])
    np.testing.assert_allclose(actual,B.T@H@B,rtol=2e-7,atol=1e-9)
    assert abs(actual[0,2])>1e-4  # The coupled terms must survive chart transport.

def test_gicp_right_chart_matches_direct_world_point_motion():
    R=Rotation.from_rotvec([.23,-.32,.41]).as_matrix();points=np.array([[2.,3.,1.],[-4.,1.,.5],[1.,-2.,3.]]);t=np.array([.8,.6]);U=np.array([[.8,-.6],[.6,.8]]);L=10.
    def skew(p):return np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]])
    J=np.vstack([np.c_[-R@skew(p),R] for p in points]);H=J.T@J
    eps=1e-6;D=[]
    for k in range(3):
        q=np.zeros(3);q[k]=eps
        def f(q):return points@(Rotation.from_rotvec([0,0,q[2]/L]).as_matrix()@R).T+np.r_[U@q[:2],0]
        D.append(((f(q)-f(-q))/(2*eps)).reshape(-1))
    B=np.column_stack(D);actual=np.asarray(gicp_information(H,R,t,L)['information'])
    np.testing.assert_allclose(actual,B.T@B,rtol=1e-7,atol=1e-9)

def test_weak_longitudinal_direction_can_leave_lateral_update_intact():
    from geometry_ieskf.state import State
    from geometry_ieskf.external_pose import ExternalPose
    x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),0);P=np.eye(15);T=np.eye(4);T[:2,3]=[10.,.1]
    observer=ExternalPose(np.zeros(3),('x','y','z','rx','ry','rz'));R=np.eye(6)*.1
    # Known synthetic geometry: along-track is unconstrained, lateral/yaw are strong.
    values,V=np.linalg.eigh(np.diag([1e-6,100.,100.]));keep=values>values[-1]*1e-3
    S=np.zeros((3,6));S[0,0]=S[1,1]=1;S[2,5]=10.;extra=np.zeros((3,6));extra[0,2]=extra[1,3]=extra[2,4]=1
    projection=np.vstack([V[:,keep].T@S,extra]);y,Q,info=observer.correct(x,P,T,R,projection);full,F,_=observer.correct(x,P,T,R)
    assert info['accepted'];assert abs(y.p[0])<1e-12
    np.testing.assert_allclose(y.p[1],full.p[1],atol=1e-12);np.testing.assert_allclose(Q[1,1],F[1,1],atol=1e-12)
    assert y.p[1]>.09  # Rejecting the whole pose would lose this valid strong component.
