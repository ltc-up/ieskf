"""Run AT from separately authorized private path bindings; no private defaults."""
import argparse,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('bindings',type=Path);p.add_argument('--reuse-raw',action='store_true');a=p.parse_args();b=json.loads(a.bindings.read_text());here=Path(__file__).resolve().parent;private=Path(b['private']);private.mkdir(parents=True,exist_ok=True)
def execute(name,args):subprocess.run([sys.executable,str(here/name),*map(str,args)],check=True)
if not a.reuse_raw:execute('extract_raw_reference.py',[b['bag'],private/'raw_reference.json'])
execute('bind_static.py',['--raw',private/'raw_reference.json','--historical-candidate',b['historical_urdf'],'--private',private,'--audit',here/'static_audit.json'])
opts={'raw':private/'reference_bound.json','motion':b['motion'],'run':b['run'],'as-npz':b['as_npz'],'gnss':b['gnss'],'config':b['historical_gnss_config'],'private':private,'output':here/'statistics.json','left':b['left'],'right':b['right']}
if (private/'visual_outline.json').exists():opts['geometry']=private/'visual_outline.json'
execute('analyze.py',[x for k,v in opts.items() for x in ('--'+k,v)])
execute('supplement.py',[x for k,v in dict(private=private,run=b['run'],gnss=b['gnss'],quality=b['quality'],output=here/'corner_quality.json',left=b['left'],right=b['right']).items() for x in ('--'+k,v)])
