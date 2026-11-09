"""Run frozen AQ2 core with an uncalibrated Native covariance factory."""
import argparse,importlib.util,json
from pathlib import Path
from geometry_ieskf.pose_uncertainty import native_pose_R
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ('input','bindings','target','deskew','deskew-service','matcher','private','output'):p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--window',default='normal');a=p.parse_args()
 spec=importlib.util.spec_from_file_location('aq2',Path(__file__).parents[1]/'phase_aq2_ndt_15d_closed_loop/run_closed_loop.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 m.pose_covariance=native_pose_R;a.mode='C';a.limit=None;a.stop_loss=.5;m.main(a)
 r=json.loads(a.output.read_text());r.update(pose_statistical_covariance='UNQUALIFIED_NATIVE_CANDIDATE',statistical_calibration=False);a.output.write_text(json.dumps(r,indent=2)+'\n')
