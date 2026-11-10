"""Own-state AY loop, additional velocity-only scan pairs; unchanged map matcher."""
import sys,os
from pathlib import Path
from ndt_global.lidar_speed import ScanVelocity
source=Path(__file__).parents[1]/'phase_ay_localization_solution/run_loop.py';sys.path.insert(0,str(source.parent));s=source.read_text();s=s.replace("ns=dict(__name__='ay_actual_loop'","ns=dict(ScanVelocity=ScanVelocity,__name__='ay_actual_loop'")
needle="ns['main'](a);r=json.loads(a.output.read_text())"
replacement="""ns['main'](a);r=json.loads(a.output.read_text())"""
# Insert only into the AR source string before AY compiles it.
marker="ns=dict(ScanVelocity=ScanVelocity,__name__='ay_actual_loop'"
insert="""
s=s.replace(' proc=None;deskproc=None;startup_ms=0.'," speed=ScanVelocity(os.environ['BB_MATCHER'],a.private/'lidar_velocity.jsonl',os.environ.get('BB_AUX','1')=='1',int(os.environ.get('BB_STRIDE','1')),json.loads(os.environ.get('BB_DROP','null')),os.environ.get('BB_FEEDBACK','0')=='1')\\n proc=None;deskproc=None;startup_ms=0.")
s=s.replace("   proc.stdin.write(json.dumps(request)+'\\\\n');", "   auxiliary_seed,aux_info=speed.seed(seed,ref,replay);request['seed_common']=auxiliary_seed.tolist();speed_info=speed.measure(path,ref,seed,replay,lever)\\n   proc.stdin.write(json.dumps(request)+'\\\\n');")
s=s.replace("ndt=json.loads(line);T=np.array(ndt['final']);", "ndt=json.loads(line);ndt['velocity_auxiliary']=aux_info;ndt['velocity_frontend_ms']=speed_info['frontend_wall_ms'];T=np.array(ndt['final']);")
s=s.replace(" if proc:proc.stdin.close();", " speed.close()\\n if proc:proc.stdin.close();")
"""
# os must exist in the generated AR namespace, which is AY's exec namespace.
s=s.replace(marker,insert+'\n'+marker);s=s.replace('ScanVelocity=ScanVelocity,__name__','ScanVelocity=ScanVelocity,os=os,__name__');s=s.replace('import argparse,json,gc,sys','import argparse,json,gc,sys,os')
exec(compile(s,str(source),'exec'),dict(__name__='bb_actual_loop',__file__=str(source),ScanVelocity=ScanVelocity))
