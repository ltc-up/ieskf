"""Final control review. Offline reference labels are imperfect proxies, never runtime input."""
import argparse,json,collections
from pathlib import Path
import numpy as np
from evaluate import stats
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();paths=json.load(open(a.root/'runtime/paths.json'));out={}
for window in ['lap','multilap']:
 ref=np.load(a.root/'runtime'/f'reference_{window}.npz');fixed=np.load(Path(paths['data'])/'phase_as_accuracy_quality'/('single_lap' if window=='lap' else 'multilap')/'evaluation.npz')
 for mode in ['matched','directional']:
  name=f'controlled2_{mode}_{window}';file=a.root/'results'/name/'frames.jsonl';counts=collections.Counter();segments={str(k):collections.Counter() for k in range(10)};energies=[];ratio=[]
  for line in file.open():
   r=json.loads(line);i=r['index'];b=next(v for v in r['branches'] if v['branch']==r['selected']);accepted=b['update']['accepted'];segment=segments[str(int(fixed['corner'][i]))]
   if ref['raw_valid_0.075'][i]:
    e=np.linalg.norm(np.asarray(b['ndt']['final'])[:2,3]-ref['G_0.075'][i,:2]);label='proxy_good' if e<=.5 else 'proxy_bad' if e>=2. else 'ambiguous';segment[label]+=1;segment[label+('_accepted' if accepted else '_rejected')]+=1
   else:segment['reference_missing']+=1
   d=b['diagnostics']
   if 'ndt' in d:
    L=np.asarray(d['ndt'][1]['information']);v,V=np.linalg.eigh(L);keep=v>max(1e-12,v[-1]*1e-3);counts[str(int(keep.sum()))]+=1;energies.append(float(np.sum(V[1,keep]**2)));ratio.append(float(v[0]/max(v[-1],1e-12)))
  out[name]=dict(retained_directions=dict(counts),lateral_energy=stats(energies),relative_minimum_eigenvalue=stats(ratio),segments={k:dict(v) for k,v in segments.items()},interpretation='L=10m; admission labels are offline engineering proxies, not true peak labels')
a.output.write_text(json.dumps(out,indent=2))
