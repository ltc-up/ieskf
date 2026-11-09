"""Paired native-time F/N comparison; explicit reference and same-prior contrasts."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.track_evaluation import Polyline,paired_errors,interpolate_reference
from ndt_global.validation import static_transform
from geometry_ieskf.pose_uncertainty import native_pose_R
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.state import State

def stats(x):return dict(rmse=float(np.sqrt(np.mean(np.asarray(x)**2))),p50=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(np.max(x)))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['F','N','input','gnss','center','output']:p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();F=json.loads((a.F/'transactions.json').read_text());N=json.loads((a.N/'transactions.json').read_text());assert [x['ref'] for x in F]==[x['ref'] for x in N];ns=np.array([v['ref'] for v in F],np.int64);origin=int(ns[0]);t=(ns-origin)*1e-9;g=list(csv.DictReader(a.gnss.open()));gt=np.array([float(v['timestamp'])-origin*1e-9 for v in g]);G,ok=interpolate_reference(gt,np.array([[float(v['base_link_'+k]) for k in 'xyz'] for v in g]),[int(v['truth_valid']) and float(v['gps_bracket_dt_sec'])<=.6 for v in g],t,.100001);c=np.genfromtxt(a.center,delimiter=',',names=True);line=Polyline(np.c_[c['Xref'],c['Yref']]);tan=line.project(G)['tangent'];lever=static_transform(json.loads(a.input.read_text())['transforms'],'base_link','imu_bosch')[:3,3];out=dict(paired_valid=int(sum(ok)),GNSS_reference='engineering75ms only',initial_seeds_identical=F[0]['seed']==N[0]['seed'],initial_Source_identical=F[0]['source_sha256']==N[0]['source_sha256'],statistical_calibration=False);direct=[]
 for name,tx in [('F',F),('N',N)]:
  xyz=[]
  for v in tx:
   d=np.array(v['update']['delta']);R=np.array(v['prior']['R'])@Rotation.from_rotvec(d[6:9]).as_matrix();xyz.append(np.array(v['prior']['p'])+d[:3]-R@lever)
  xyz=np.asarray(xyz);es,ed=paired_errors(xyz,G,tan);out[name]=dict(XY=stats(np.linalg.norm((xyz-G)[ok,:2],axis=1)),abs_lateral=stats(abs(ed[ok])))
 for v in F:
  x=State(np.asarray(v['prior']['p']),np.zeros(3),np.asarray(v['prior']['R']),np.zeros(3),np.zeros(3));R=native_pose_R(v['ndt'],('x','y','z','rx','ry','rz'),x);_,_,u=ExternalPose(lever,('x','y','z','rx','ry','rz')).correct(x,np.asarray(v['prior']['P']),v['ndt']['final'],R)
  if u['accepted']:direct.append((np.linalg.norm(v['update']['delta'][:3]),np.linalg.norm(u['delta'][:3])))
 out['same_F_prior_direct_R_effect']=dict(frames=len(direct),F_median_delta_position=float(np.median(np.array(direct)[:,0])),N_median_delta_position=float(np.median(np.array(direct)[:,1])),interpretation='gain effect only; zero dummy dynamics means used only because pose H/residual independent of v/bias means, not claimed real trajectories')
 a.output.write_text(json.dumps(out,indent=2)+'\n')
