"""Private per-region coverage audit; no semantic or independent-sensor claims."""
import sys,json,struct
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);plan={r['id']:r for r in json.load(open(root/'inputs/geometric_diagnostic_plan.json'))};out=[]
for r in map(json.loads,open(root/'results/native_raw_geometry.jsonl')):
 if not r['points']:continue
 p=plan[r['id']];f=open(p['source'],'rb');n=struct.unpack('<Q',f.read(8))[0];a=np.fromfile(f,'<f4').reshape(-1,3);f.close();assert n==len(a);mask=np.zeros(n,bool)
 for q in r['points']:mask[q['i']]=q['associated']
 def coverage(z):
  if not len(z):return dict(n=0)
  radius=np.linalg.norm(z[:,:2],axis=1);angle=np.arctan2(z[:,1],z[:,0]);return dict(n=len(z),bounds_min=z.min(0).tolist(),bounds_max=z.max(0).tolist(),range_bins=np.histogram(radius,[0,10,30,60,np.inf])[0].tolist(),angle_bins=np.histogram(angle,np.linspace(-np.pi,np.pi,13))[0].tolist(),z_bins=np.histogram(z[:,2],[-np.inf,0,2,5,np.inf])[0].tolist(),occupied_2m_cells=len(np.unique(np.floor(z/2),axis=0)))
 out.append(dict(id=r['id'],all=coverage(a),associated=coverage(a[mask]),unassociated=coverage(a[~mask]),no_semantic_labels=True))
(root/'results/spatial_coverage.json').write_text(json.dumps(out,indent=2));print(len(out),'detailed spatial coverage records')
