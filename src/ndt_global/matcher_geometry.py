"""Full local pose-geometry diagnostics in chosen common-anchor axes.
Translation(m), 10*rotation(rad); orientation rotates about base, not map origin.
No new measurement covariance or correct-peak probability is produced.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.pose_uncertainty import euler_xyz_to_body

def ndt_native_information(row,length=10.,anchor_rotation=None):
 C=np.asarray(row['native_internal'],float);alpha=np.asarray(row['parameters_internal']).reshape(6)[3:];R=Rotation.from_euler('XYZ',alpha).as_matrix();J=euler_xyz_to_body(alpha);A=np.zeros((6,6));A[:3,:3]=R;A[3:,3:]=np.linalg.inv(J)/length
 # Shape proxy only: logged kappa is evaluated at rounded float pose, whereas
 # native C is formed at internal parameters. The scalar cancels in trace/axis
 # shape comparisons; exact original GN magnitude is not reconstructible.
 info=float(row['kappa_at_float_pose'])*np.linalg.inv(C)
 out=A.T@info@A
 return transport(out,R,anchor_rotation)

def gicp_information(row,length=10.,anchor_rotation=None):
 H=np.asarray(row['H_right_rotation_translation'],float);B=np.zeros((6,6));B[:3,3:]=np.eye(3)/length;B[3:,:3]=np.eye(3)
 return transport(B.T@H@B/max(row['source_n'],1),np.asarray(row['final'])[:3,:3] if anchor_rotation is not None else np.eye(3),anchor_rotation)

def transport(I,pose_rotation,anchor_rotation):
 if anchor_rotation is None:return I
 Q=np.zeros((6,6));Q[:3,:3]=pose_rotation.T@anchor_rotation;Q[3:,3:]=pose_rotation.T@anchor_rotation
 return Q.T@I@Q

def describe(I):
 I=np.asarray(I,float);I=(I+I.T)/2
 if I.shape!=(6,6) or not np.isfinite(I).all():raise ValueError('INVALID_GEOMETRY_MATRIX')
 vals,V=np.linalg.eigh(I);trace=float(np.trace(I))
 if vals[0]<=0 or trace<=0:raise ValueError('NON_POSITIVE_LOCAL_INFORMATION')
 D=np.sqrt(np.diag(I));cor=I/(D[:,None]*D[None,:]);marginal=I[:3,:3]-I[:3,3:]@np.linalg.solve(I[3:,3:],I[3:,:3]);marginal=(marginal+marginal.T)/2
 return dict(trace_normalized=I/trace,eigenvalues_normalized=vals/trace,eigenvectors=V,condition=float(vals[-1]/vals[0]),conditional_axis_energy=np.diag(I)/trace,marginal_translation_energy=np.diag(marginal)/np.trace(marginal),normalized_coupling=cor,weakest_vector=V[:,0],marginal_translation_information=marginal,full_information=I)

def compare(A,B):
 a=describe(A);b=describe(B);return dict(trace_shape_cosine=float(np.sum(a['trace_normalized']*b['trace_normalized'])/(np.linalg.norm(a['trace_normalized'])*np.linalg.norm(b['trace_normalized']))),weakest_axis_abs_cosine=float(abs(a['weakest_vector']@b['weakest_vector'])),weakest2_subspace_cosines=np.linalg.svd(a['eigenvectors'][:,:2].T@b['eigenvectors'][:,:2],compute_uv=False))
