"""Sequential GPU comparisons; identities/arguments are explicitly supplied."""
import argparse,subprocess,sys,os
from pathlib import Path
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--bindings',required=True);p.add_argument('--target',required=True);p.add_argument('--deskew',required=True);p.add_argument('--deskew-service',required=True);p.add_argument('--matcher',required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--public',type=Path,required=True);p.add_argument('--modes',default='ABDE');a=p.parse_args()
 for mode in a.modes:
  cmd=[sys.executable,str(Path(__file__).with_name('run_closed_loop.py'))]
  for key in ['input','bindings','target','deskew','deskew_service','matcher']:cmd +=['--'+key.replace('_','-'),getattr(a,key)]
  cmd+=['--private',str(a.private/mode),'--output',str(a.public/(mode+'_results.json')),'--mode',mode]
  subprocess.run(cmd,check=True)
