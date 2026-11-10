"""Fixed raw-sensor corruption before frontend, not a postfiltered fake signal."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists();d=json.loads(a.input.read_text());origin=min(e['header_ns'] for e in d['events'])+1_000_000_000
for key in ['events','raw_topics']:
 if key=='events':rows=d['events'];stamp='header_ns';field='velocity'
 else:rows=d['raw_topics']['/sensor/kistler/measurement'];stamp='header_ns';field='raw'
 out=[]
 for e in rows:
  t=(e[stamp]-origin)*1e-9
  if 20<=t<22:continue
  e=dict(e)
  if 35<=t<35.5:e[field]=[e[field][0]+15,e[field][1]]
  out.append(e)
 if key=='events':d[key]=out
 else:d[key]['/sensor/kistler/measurement']=out
a.output.write_text(json.dumps(d,separators=(',',':')))
