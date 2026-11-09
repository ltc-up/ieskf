"""Compare same native-reference prefixes; never compare variable arrival times."""
import argparse,json
from pathlib import Path
import numpy as np
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--first',type=Path,required=True);p.add_argument('--second',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();x=json.loads(a.first.read_text());y=json.loads(a.second.read_text());assert y and len(x)>=len(y);x=x[:len(y)];assert all(i['ref']==j['ref'] for i,j in zip(x,y))
 def diff(get):return max(float(np.max(abs(np.asarray(get(i))-np.asarray(get(j))))) for i,j in zip(x,y))
 z=dict(frames=len(y),reference_stamps_equal=True,seed_max_abs=diff(lambda v:v['seed']),matching_final_max_abs=diff(lambda v:v['ndt']['final']),update_delta_max_abs=diff(lambda v:v['update']['delta']),all_acceptances_equal=all(i['update']['accepted']==j['update']['accepted'] for i,j in zip(x,y)),comparison='same native reference, not variable-latency publication timestamps');a.output.write_text(json.dumps(z,indent=2)+'\n')
