"""Predeclared sensor fault scenarios; never selects a corner/reference frame."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--record',type=Path,required=True);a=p.parse_args();assert not a.output.exists();e=json.loads(a.input.read_text());origin=min(z['header_ns'] for z in e['events'])+1_000_000_000;rows=[]
for z in e['events']:
 t=(z['header_ns']-origin)*1e-9
 if 20<=t<22:continue
 q=dict(z)
 if 35<=t<35.5:q['velocity']=[q['velocity'][0]+15,q['velocity'][1]]
 rows.append(q)
e['events']=rows;a.output.write_text(json.dumps(e,separators=(',',':')));a.record.write_text(json.dumps(dict(dropout_s=[20,22],valid_status_additive_X_m_s=15,corrupt_interval_s=[35,35.5],origin='first extracted header plus1s; fixed before fault run; no GNSS/segment input')))
