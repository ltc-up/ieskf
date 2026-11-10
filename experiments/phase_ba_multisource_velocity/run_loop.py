"""Actual AY/AR sensor/CUDA loop with only the declared frontend changed."""
import os,sys
from pathlib import Path
source=Path(__file__).parents[1]/'phase_ay_localization_solution/run_loop.py';sys.path.insert(0,str(source.parent));s=source.read_text();mode=os.environ.get('BA_MODE','F');assert mode in ['F','B','J','W']
needle="a.mode='F';a.stop_loss=2."
replacement="from ndt_global.frontend_replay import FrontendReplay\nFrontendReplay.frontend_mode="+repr(mode)+"\nns['PoseDelayReplay']=FrontendReplay\na.mode='F';a.stop_loss=2."
if mode=='J':
 replacement="from ndt_global.joint_velocity_replay import JointVelocityReplay\nfrom geometry_ieskf.velocity_bias import BiasOptical,BiasPose\nns['PoseDelayReplay']=JointVelocityReplay;ns['OpticalVelocity']=BiasOptical;ns['ExternalPose']=BiasPose\na.mode='F';a.stop_loss=2."
 s=s.replace("prior=dict(calibration_yaw_rad=float(getattr(prior,'beta',0.)),","prior=dict(optical_bias=getattr(prior,'bo',np.zeros(2)).tolist(),calibration_yaw_rad=float(getattr(prior,'beta',0.)),")
if mode=='W':replacement="from ndt_global.brake_counterfactual import BrakeWheelCounterfactual\nns['PoseDelayReplay']=BrakeWheelCounterfactual\na.mode='F';a.stop_loss=2."
assert needle in s;s=s.replace(needle,replacement);exec(compile(s,str(source),'exec'),dict(__name__='ba_actual_loop',__file__=str(source)))
