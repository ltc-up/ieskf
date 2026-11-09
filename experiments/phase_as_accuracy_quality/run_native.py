"""Exploratory causal Native-R branch; immutable AR full-pose dynamics/matcher."""
import argparse,importlib.util,json
from pathlib import Path
from geometry_ieskf.pose_uncertainty import native_pose_R
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--window',default='lap');a=p.parse_args();spec=importlib.util.spec_from_file_location('ar',Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.pose_covariance=native_pose_R;a.mode='F';a.limit=None;a.stop_loss=None;m.main(a);v=json.loads(a.output.read_text());v.update(pose_statistical_covariance='UNCALIBRATED_NATIVE_CHART_TRANSPORT',experiment='genuine causal feedback, not frozen pose replay');a.output.write_text(json.dumps(v,indent=2)+'\n')
