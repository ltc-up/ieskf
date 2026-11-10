"""CLI launcher for immutable CUDA/Deskew interfaces; no evaluation inputs."""
import argparse,os,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--root',type=Path,required=True);p.add_argument('--window',choices=['lap','multilap'],default='lap');p.add_argument('--limit',type=int);p.add_argument('--optical-file',type=Path);p.add_argument('--managed',action='store_true');p.add_argument('--profile',action='store_true');p.add_argument('--speed-norm',action='store_true');a=p.parse_args();a.root.mkdir(exist_ok=True,parents=True);assert not (a.root/a.name).exists() and not (a.root/(a.name+'.log')).exists(),'EXISTING_RUN_PRESERVED'
b='data/phase_ar_loop44_map0105/bindingslap.json' if a.window=='lap' else 'data/phase_as_accuracy_quality/bindings_multilap.json';r='data/phase_ar_loop44_map0105/runtime/';args=[sys.executable,'experiments/phase_az_velocity_fusion/run_loop.py','--input','data/phase_ao_ap_validation/loop44.json','--bindings',b,'--target','data/phase_ar_loop44_map0105/map0105_target.bin','--deskew',r+'deskew_service','--deskew-service',r+'deskew_service','--matcher',r+'match_fixed100','--optical',str(a.optical_file or a.root/'optical.json'),'--private',str(a.root/a.name),'--output',str(a.root/(a.name+'_summary.json')),'--policy','optical_xy','--window',a.window]
if a.limit:args+=['--limit',str(a.limit)]
if a.profile:args+=['--profile']
env=os.environ.copy();env.pop('AZ_MANAGED',None);env.pop('AZ_SPEED_NORM',None)
if a.managed:env['AZ_MANAGED']='1'
if a.speed_norm:
 if not a.managed:raise ValueError('PLANAR_SPEED_REQUIRES_MANAGED_SOURCE')
 env['AZ_SPEED_NORM']='1'
with (a.root/(a.name+'.log')).open('w') as log:subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,check=True,env=env)
