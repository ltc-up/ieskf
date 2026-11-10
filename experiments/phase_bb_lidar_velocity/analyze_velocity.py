"""Private paired speed series and public non-reconstructive source statistics."""
import argparse,json
from pathlib import Path
import numpy as np
from geometry_ieskf.lidar_velocity import body_twist,sensor_twist_velocity,estimate_pair
from ndt_global.validation import static_transform
p=argparse.ArgumentParser();p.add_argument('--pairs',type=Path,required=True);p.add_argument('--optical',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();d=json.loads(Path('data/phase_ao_ap_validation/loop44.json').read_text());raw=json.loads(a.optical.read_text())['raw_topics']['/sensor/kistler/measurement'];ot=np.array([r['header_ns'] for r in raw]);ov=np.array([r['raw'] for r in raw]);T=static_transform(d['transforms'],'base_link','kistler');w=[e for e in d['events'] if e['kind']=='wheel'];wt=np.array([e['header_ns'] for e in w]);wv=np.array([e['speed'] for e in w]);out={};rows=[]
def stats(x):
 x=np.asarray(x);return dict(n=len(x),mean=np.mean(x,axis=0).tolist(),RMS=np.sqrt(np.mean(x*x,axis=0)).tolist(),absP95=np.percentile(abs(x),95,axis=0).tolist(),max=np.max(abs(x),axis=0).tolist())
for line in a.pairs.open():
 r=json.loads(line);mid=r['header_ns']-int(r['dt']*.5e9);v=body_twist(r['final'],r['dt']);pred=sensor_twist_velocity(v,T)[:2];ix=(ot>=r['header_ns']-r['dt']*1e9)&(ot<=r['header_ns']);optic=ov[ix].mean(axis=0) if ix.any() else np.full(2,np.nan);wheel=np.interp(mid-wt[0],wt-wt[0],wv);status='VALID'
 try:_,g=estimate_pair(r,r['dt'])
 except ValueError as e:status=str(e);g={}
 rows.append(dict(index=r['index'],lag=r['lag'],header_ns=r['header_ns'],twist=v.tolist(),optical=optic.tolist(),sensor_prediction=pred.tolist(),wheel=wheel,quality=status,geometry=g,pair_wall_ms=r['pair_wall_ms'],iterations=r.get('iterations')))
for lag in [1,2]:
 z=[r for r in rows if r['lag']==lag];valid=[r for r in z if r['quality']=='VALID'];x=np.array([np.array(r['sensor_prediction'])-r['optical'] for r in valid]);out[str(lag)]=dict(total=len(z),valid=len(valid),optical_difference=stats(x),wheel_difference=stats([r['twist'][0]-r['wheel'] for r in valid]),pair_ms_quantiles=np.percentile([r['pair_wall_ms'] for r in z],[50,95,99,100]).tolist(),quality_counts={k:sum(r['quality']==k for r in z) for k in set(r['quality'] for r in z)},disagreement3m_s=int(sum(max(abs(e))>3 for e in x)))
a.out.write_text(json.dumps(out,indent=2)+'\n');a.out.with_suffix('.private.json').write_text(json.dumps(rows));print(json.dumps(out,indent=2))
