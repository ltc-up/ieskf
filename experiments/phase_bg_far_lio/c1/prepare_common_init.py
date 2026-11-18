"""Authorized ONE recorded EKF mean, both header and availability <=fixed135s.
No future sample, historical covariance/bias/velocity copying or candidate selection.
"""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from continuous_inputs import ORIGIN,INIT

def main(a):
 from rclpy.serialization import deserialize_message
 from nav_msgs.msg import Odometry
 x=json.load(open(a.motion));e=x['events'];v=max((v for v in e if v['kind']=='pose' and v['valid'] and v['header_ns']<=INIT and v['recorder_ns']<=INIT),key=lambda v:v['header_ns'])
 db=sqlite3.connect('file:'+a.bag+'?mode=ro&immutable=1',uri=True);msg=deserialize_message(db.execute('select data from messages where id=?',(v['message_id'],)).fetchone()[0],Odometry)
 if msg.header.frame_id!='map' or msg.child_frame_id!='base_link':raise ValueError('RECORDED_POSE_REFERENCE_POINT_CONTRACT')
 q=msg.pose.pose.orientation;p=msg.pose.pose.position;R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T=np.eye(4);T[:3,:3]=R;T[:3,3]=[p.x,p.y,p.z]
 im=[z for z in e if z['kind']=='imu' and INIT-5_000_000_000<=z['header_ns']<=INIT and z['recorder_ns']<=INIT and z['valid']];wh=[z for z in e if z['kind']=='wheel' and INIT-5_000_000_000<=z['header_ns']<=INIT and z['recorder_ns']<=INIT and z['valid']]
 g=np.array([z['omega'] for z in im]);f=np.array([z['force'] for z in im]);s=np.array([z['speed'] for z in wh]);Tbi=np.array(json.load(open(a.extrinsics))['T_base_imu']);fb=f@Tbi[:3,:3].T;pred=R.T@np.array([0.,0.,9.80665]);angle=float(np.degrees(np.arccos(np.clip(np.dot(fb.mean(0),pred)/np.linalg.norm(fb.mean(0))/np.linalg.norm(pred),-1,1))))
 if len(im)<900 or len(wh)<450 or abs(s).max()>.01 or np.linalg.norm(g,axis=1).max()>.03 or f.std(0).max()>.5 or np.diff([z['header_ns'] for z in im]).max()>25_000_000:raise ValueError('CAUSAL_STATIONARITY_FAILURE')
 out=Path(a.root)/'recorded_common_init';out.mkdir(exist_ok=False)
 result=dict(init_ns=INIT,mean=dict(T_map_base=T.tolist(),position=T[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()),source={k:v[k] for k in ['message_id','header_ns','recorder_ns','topic','frame']},child_frame=msg.child_frame_id,recorded_pose_covariance_not_used=list(msg.pose.covariance),sample_interval=[im[0]['header_ns'],im[-1]['header_ns']],stationarity=dict(imus=len(im),wheels=len(wh),wheel_abs_max=float(abs(s).max()),gyro_mean=g.mean(0).tolist(),gyro_std=g.std(0).tolist(),force_mean=f.mean(0).tolist(),force_std=f.std(0).tolist(),gravity_angle_deg=angle),velocity_initial_condition='zero at base conditional on verified pit stationary samples',bias='native existing zero means; no fitted future or mature biases',classification='external-initial-pose conditional localization; not independent global GNSS-free initialization')
 (out/'initialization.json').write_text(json.dumps(result,indent=2));print('one initial mean frozen',v['message_id'],'gravity agreement degrees',angle)
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','bag','motion','extrinsics']];main(p.parse_args())
