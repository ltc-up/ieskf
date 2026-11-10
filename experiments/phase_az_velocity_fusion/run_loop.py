"""Reuse AY true CUDA loop; sensor-axis ablation and managed source opt-ins."""
import sys,os
from pathlib import Path
source=Path(__file__).parents[1]/'phase_ay_localization_solution/run_loop.py'
sys.path.insert(0,str(source.parent))
s=source.read_text()
# The CLI optical_xy tag retains source-substitution semantics, not observed axes.
s=s.replace("(0,1) if a.policy=='optical_xy' else (1,)","(0,) if a.policy=='optical_xy' else (1,)")
if os.environ.get('AZ_MANAGED')=='1':
 s=s.replace("a.mode='F';a.stop_loss=2.","from ndt_global.velocity_manager import ManagedVelocityReplay\nns['PoseDelayReplay']=ManagedVelocityReplay\na.mode='F';a.stop_loss=2.")
 s=s.replace("replay.policy=a.policy;","replay.manager_log=Journal(a.private/'manager.jsonl',True);replay.policy=a.policy;")
if os.environ.get('AZ_SPEED_NORM')=='1':
 s=s.replace('from geometry_ieskf.optical_velocity import OpticalVelocity','from geometry_ieskf.planar_speed import OpticalPlanarSpeed as OpticalVelocity')
# Execute as the original phase file so its restored AR/AW relative paths stay valid.
ns=dict(__name__='az_actual_loop',__file__=str(source));exec(compile(s,str(source),'exec'),ns)
